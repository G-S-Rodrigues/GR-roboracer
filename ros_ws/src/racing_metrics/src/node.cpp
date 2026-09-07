#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <racing_interfaces/msg/safety_status.hpp>
#include <racing_interfaces/msg/scenario_metrics.hpp>
#include <racing_interfaces/msg/track_relative_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "racing_metrics/metrics_accumulator.hpp"

namespace racing_metrics {
namespace {

constexpr auto metrics_frame = "map";
constexpr auto metrics_source = "racing_metrics";

double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
    constexpr double nanoseconds_per_second = 1000000000.0;
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) / nanoseconds_per_second;
}

void write_atomically(const std::filesystem::path &path,
                      const std::string &content) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    auto temporary_path = path;
    temporary_path += ".tmp";
    {
        std::ofstream output{temporary_path, std::ios::out | std::ios::trunc};
        if (!output) {
            throw std::runtime_error("cannot open metrics output: " +
                                     temporary_path.string());
        }
        output << content << '\n';
        output.flush();
        if (!output) {
            throw std::runtime_error("cannot write metrics output: " +
                                     temporary_path.string());
        }
    }
    std::filesystem::rename(temporary_path, path);
}

}  // namespace

class MetricsNode : public rclcpp::Node {
   public:
    MetricsNode() : Node("racing_metrics") {
        RunProvenance provenance;
        provenance.source = metrics_source;
        provenance.scenario_id =
            declare_parameter("scenario_id", std::string{"baseline"});
        provenance.seed = static_cast<std::uint64_t>(
            declare_parameter<std::int64_t>("seed", 42));
        provenance.timestep_ratio = static_cast<std::uint32_t>(
            declare_parameter<std::int64_t>("timestep_ratio", 1));
        provenance.code_revision =
            declare_parameter("code_revision", std::string{"unknown"});
        provenance.container_image_digest =
            declare_parameter("container_image_digest", std::string{"unknown"});
        provenance.track_version =
            declare_parameter("track_version", std::string{"unknown"});
        provenance.vehicle_parameter_version = declare_parameter(
            "vehicle_parameter_version", std::string{"unknown"});
        output_path_ = declare_parameter(
            "output_path", std::string{"/tmp/racing_metrics.json"});
        collision_threshold_ = declare_parameter("collision_threshold", 0.01);
        const auto track_length = declare_parameter("track_length", 31.4159);
        if (provenance.timestep_ratio == 0U) {
            throw std::invalid_argument("timestep_ratio must be positive");
        }
        if (!std::isfinite(collision_threshold_) ||
            collision_threshold_ < 0.0) {
            throw std::invalid_argument(
                "collision_threshold must be finite and nonnegative");
        }
        accumulator_ = std::make_unique<MetricsAccumulator>(
            std::move(provenance), track_length);

        metrics_publisher_ =
            create_publisher<racing_interfaces::msg::ScenarioMetrics>(
                "/scenario/metrics",
                rclcpp::QoS(1).reliable().transient_local());
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &message) {
                update_clearance(*message);
            });
        safety_subscription_ =
            create_subscription<racing_interfaces::msg::SafetyStatus>(
                "/safety/status", rclcpp::QoS(10).reliable(),
                [this](
                    const racing_interfaces::msg::SafetyStatus::ConstSharedPtr
                        &message) { update_saturation(*message); });
        state_subscription_ =
            create_subscription<racing_interfaces::msg::TrackRelativeState>(
                "/ground_truth/track_relative_state",
                rclcpp::QoS(10).reliable(),
                [this](const racing_interfaces::msg::TrackRelativeState::
                           ConstSharedPtr &message) { observe(*message); });
        finalize_service_ = create_service<std_srvs::srv::Trigger>(
            "~/finalize",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr &,
                const std_srvs::srv::Trigger::Response::SharedPtr &response) {
                finalize(response);
            });
    }

   private:
    void update_clearance(const sensor_msgs::msg::LaserScan &message) {
        auto minimum = std::numeric_limits<double>::infinity();
        for (const auto range : message.ranges) {
            if (std::isfinite(range) && range >= 0.0F) {
                minimum = std::min(minimum, static_cast<double>(range));
            }
        }
        if (std::isfinite(minimum)) {
            latest_clearance_ = minimum;
            has_clearance_ = true;
        }
    }

    void update_saturation(
        const racing_interfaces::msg::SafetyStatus &message) {
        constexpr std::uint32_t saturation_mask =
            racing_interfaces::msg::SafetyStatus::CLAMP_SPEED |
            racing_interfaces::msg::SafetyStatus::CLAMP_STEERING |
            racing_interfaces::msg::SafetyStatus::CLAMP_ACCELERATION |
            racing_interfaces::msg::SafetyStatus::CLAMP_STEERING_RATE;
        latest_saturated_ = (message.active_clamps & saturation_mask) != 0U;
    }

    void observe(const racing_interfaces::msg::TrackRelativeState &message) {
        if (!has_clearance_ || published_) {
            return;
        }
        accumulator_->observe({stamp_seconds(message.header.stamp), message.s,
                               latest_clearance_, std::abs(message.d),
                               latest_clearance_ <= collision_threshold_,
                               latest_saturated_});
        if (accumulator_->summary().lap_completed) {
            publish_and_write();
        }
    }

    racing_interfaces::msg::ScenarioMetrics make_message(
        const ScenarioMetricsData &metrics, const rclcpp::Time &stamp) const {
        racing_interfaces::msg::ScenarioMetrics message;
        message.header.stamp = stamp;
        message.header.frame_id = metrics_frame;
        message.source = metrics.source;
        message.scenario_id = metrics.scenario_id;
        message.seed = metrics.seed;
        message.timestep_ratio = metrics.timestep_ratio;
        message.lap_completed = metrics.lap_completed;
        message.lap_time = metrics.lap_time;
        message.collision_count = metrics.collision_count;
        message.minimum_wall_clearance = metrics.minimum_wall_clearance;
        message.maximum_tracking_error = metrics.maximum_tracking_error;
        message.p95_tracking_error = metrics.p95_tracking_error;
        message.control_saturation_events = metrics.control_saturation_events;
        message.code_revision = metrics.code_revision;
        message.container_image_digest = metrics.container_image_digest;
        message.track_version = metrics.track_version;
        message.vehicle_parameter_version = metrics.vehicle_parameter_version;
        return message;
    }

    void publish_and_write() {
        const auto stamp = get_clock()->now();
        const auto metrics = accumulator_->summary();
        metrics_publisher_->publish(make_message(metrics, stamp));
        write_atomically(output_path_,
                         to_json(metrics, stamp.nanoseconds(), metrics_frame));
        published_ = true;
    }

    void finalize(const std_srvs::srv::Trigger::Response::SharedPtr &response) {
        try {
            if (!published_) {
                publish_and_write();
            }
            response->success = true;
            response->message = output_path_.string();
        } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
        }
    }

    std::unique_ptr<MetricsAccumulator> accumulator_;
    std::filesystem::path output_path_;
    double collision_threshold_{0.0};
    double latest_clearance_{0.0};
    bool has_clearance_{false};
    bool latest_saturated_{false};
    bool published_{false};
    rclcpp::Publisher<racing_interfaces::msg::ScenarioMetrics>::SharedPtr
        metrics_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::SafetyStatus>::SharedPtr
        safety_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::TrackRelativeState>::SharedPtr
        state_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_service_;
};

}  // namespace racing_metrics

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<racing_metrics::MetricsNode>());
    rclcpp::shutdown();
    return 0;
}
