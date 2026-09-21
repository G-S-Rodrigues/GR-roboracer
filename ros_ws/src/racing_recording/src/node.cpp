#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <racing_interfaces/msg/localization_error.hpp>
#include <racing_interfaces/msg/safety_status.hpp>
#include <racing_interfaces/msg/scenario_metrics.hpp>
#include <racing_interfaces/msg/track_relative_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "racing_recording/event_recorder.hpp"

namespace racing_recording {
namespace {

constexpr auto ground_truth_odometry_topic = "/ground_truth/odom";

std::int64_t timestamp_nanoseconds(const builtin_interfaces::msg::Time &stamp) {
    constexpr std::int64_t nanoseconds_per_second = 1000000000;
    return static_cast<std::int64_t>(stamp.sec) * nanoseconds_per_second +
           static_cast<std::int64_t>(stamp.nanosec);
}

std::string escape_json(std::string_view value) {
    std::ostringstream output;
    for (const auto character : value) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << character;
                break;
        }
    }
    return output.str();
}

template <typename Message>
std::int64_t message_timestamp(const Message &message) {
    return timestamp_nanoseconds(message.header.stamp);
}

std::string drive_payload(
    const ackermann_msgs::msg::AckermannDriveStamped &message) {
    std::ostringstream output;
    output << std::setprecision(17) << "{\"frame_id\":\""
           << escape_json(message.header.frame_id)
           << "\",\"speed\":" << message.drive.speed
           << ",\"steering_angle\":" << message.drive.steering_angle
           << ",\"acceleration\":" << message.drive.acceleration
           << ",\"steering_angle_velocity\":"
           << message.drive.steering_angle_velocity << '}';
    return output.str();
}

// JSON has no NaN: an unavailable localization sample's errors are NaN by
// contract (LocalizationError.msg), and must stay parseable as null.
std::string json_number(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    std::ostringstream output;
    output << std::setprecision(17) << value;
    return output.str();
}

std::string pose_payload(const nav_msgs::msg::Odometry &message) {
    const auto &pose = message.pose.pose;
    std::ostringstream output;
    output << std::setprecision(17) << "{\"frame_id\":\""
           << escape_json(message.header.frame_id)
           << "\",\"x\":" << pose.position.x << ",\"y\":" << pose.position.y
           << ",\"qx\":" << pose.orientation.x
           << ",\"qy\":" << pose.orientation.y
           << ",\"qz\":" << pose.orientation.z
           << ",\"qw\":" << pose.orientation.w << '}';
    return output.str();
}

std::string localization_error_payload(
    const racing_interfaces::msg::LocalizationError &message) {
    std::ostringstream output;
    output << "{\"source\":\"" << escape_json(message.source)
           << "\",\"estimate_topic\":\"" << escape_json(message.estimate_topic)
           << "\",\"available\":" << (message.available ? "true" : "false")
           << ",\"position_error\":" << json_number(message.position_error)
           << ",\"lateral_error\":" << json_number(message.lateral_error)
           << ",\"longitudinal_error\":"
           << json_number(message.longitudinal_error)
           << ",\"heading_error\":" << json_number(message.heading_error)
           << ",\"sample_count\":" << message.sample_count
           << ",\"available_count\":" << message.available_count
           << ",\"position_rmse\":" << json_number(message.position_rmse)
           << ",\"position_maximum\":" << json_number(message.position_maximum)
           << '}';
    return output.str();
}

std::string state_payload(
    const racing_interfaces::msg::TrackRelativeState &message) {
    std::ostringstream output;
    output << std::setprecision(17) << "{\"source\":\""
           << escape_json(message.source) << "\",\"s\":" << message.s
           << ",\"d\":" << message.d
           << ",\"heading_error\":" << message.heading_error
           << ",\"v_s\":" << message.v_s << ",\"v_d\":" << message.v_d << '}';
    return output.str();
}

std::string safety_payload(
    const racing_interfaces::msg::SafetyStatus &message) {
    std::ostringstream output;
    output << "{\"source\":\"" << escape_json(message.source)
           << "\",\"active_clamps\":" << message.active_clamps
           << ",\"reason\":" << static_cast<unsigned int>(message.reason)
           << ",\"heartbeat\":" << message.heartbeat << '}';
    return output.str();
}

std::string metrics_payload(
    const racing_interfaces::msg::ScenarioMetrics &message) {
    std::ostringstream output;
    output << std::setprecision(17) << "{\"source\":\""
           << escape_json(message.source) << "\",\"scenario_id\":\""
           << escape_json(message.scenario_id) << "\",\"seed\":" << message.seed
           << ",\"timestep_ratio\":" << message.timestep_ratio
           << ",\"lap_completed\":"
           << (message.lap_completed ? "true" : "false")
           << ",\"lap_time\":" << message.lap_time
           << ",\"collision_count\":" << message.collision_count
           << ",\"minimum_wall_clearance\":" << message.minimum_wall_clearance
           << ",\"maximum_tracking_error\":" << message.maximum_tracking_error
           << ",\"p95_tracking_error\":" << message.p95_tracking_error
           << ",\"control_saturation_events\":"
           << message.control_saturation_events << ",\"code_revision\":\""
           << escape_json(message.code_revision)
           << "\",\"container_image_digest\":\""
           << escape_json(message.container_image_digest)
           << "\",\"track_version\":\"" << escape_json(message.track_version)
           << "\",\"vehicle_parameter_version\":\""
           << escape_json(message.vehicle_parameter_version) << "\"}";
    return output.str();
}

}  // namespace

class RecordingNode : public rclcpp::Node {
   public:
    RecordingNode() : Node("racing_recording") {
        const auto seed = declare_parameter<std::int64_t>("seed", 42);
        const auto timestep_ratio =
            declare_parameter<std::int64_t>("timestep_ratio", 1);
        if (seed < 0 || timestep_ratio <= 0) {
            throw std::invalid_argument(
                "seed must be nonnegative and timestep_ratio positive");
        }
        RecordingManifest manifest;
        manifest.source = get_name();
        manifest.scenario_id =
            declare_parameter("scenario_id", std::string{"baseline"});
        manifest.seed = static_cast<std::uint64_t>(seed);
        manifest.timestep_ratio = static_cast<std::uint32_t>(timestep_ratio);
        manifest.code_revision =
            declare_parameter("code_revision", std::string{"unknown"});
        manifest.container_image_digest =
            declare_parameter("container_image_digest", std::string{"unknown"});
        manifest.track_version =
            declare_parameter("track_version", std::string{"unknown"});
        manifest.vehicle_parameter_version = declare_parameter(
            "vehicle_parameter_version", std::string{"unknown"});
        manifest.controller_configuration = declare_parameter(
            "controller_configuration", std::string{"unknown"});
        manifest.platform =
            declare_parameter("platform", std::string{"unknown"});
        const auto output_path = declare_parameter(
            "output_path", std::string{"/tmp/racing_run.jsonl"});
        recorder_ =
            std::make_unique<EventRecorder>(output_path, std::move(manifest));

        drive_subscription_ =
            create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", rclcpp::QoS(10).reliable(),
                [this](const ackermann_msgs::msg::AckermannDriveStamped::
                           ConstSharedPtr &message) {
                    recorder_->record(
                        "/drive", "ackermann_msgs/msg/AckermannDriveStamped",
                        message_timestamp(*message), drive_payload(*message));
                });
        state_subscription_ =
            create_subscription<racing_interfaces::msg::TrackRelativeState>(
                "/ground_truth/track_relative_state",
                rclcpp::QoS(10).reliable(),
                [this](const racing_interfaces::msg::TrackRelativeState::
                           ConstSharedPtr &message) {
                    recorder_->record(
                        "/ground_truth/track_relative_state",
                        "racing_interfaces/msg/TrackRelativeState",
                        message_timestamp(*message), state_payload(*message));
                });
        safety_subscription_ =
            create_subscription<racing_interfaces::msg::SafetyStatus>(
                "/safety/status", rclcpp::QoS(10).reliable(),
                [this](
                    const racing_interfaces::msg::SafetyStatus::ConstSharedPtr
                        &message) {
                    recorder_->record(
                        "/safety/status", "racing_interfaces/msg/SafetyStatus",
                        message_timestamp(*message), safety_payload(*message));
                });
        metrics_subscription_ = create_subscription<
            racing_interfaces::msg::ScenarioMetrics>(
            "/scenario/metrics", rclcpp::QoS(1).reliable().transient_local(),
            [this](const racing_interfaces::msg::ScenarioMetrics::ConstSharedPtr
                       &message) {
                recorder_->record("/scenario/metrics",
                                  "racing_interfaces/msg/ScenarioMetrics",
                                  message_timestamp(*message),
                                  metrics_payload(*message));
                recorder_->flush();
            });
        // The estimate and error streams racing_evaluation scores, plus the
        // ground truth it scores against: enough for
        // scripts/compare_localization.py to rescore a run offline.
        const auto estimate_topic = declare_parameter(
            "estimate_topic", std::string{ground_truth_odometry_topic});
        truth_subscription_ = subscribe_pose(ground_truth_odometry_topic,
                                             rclcpp::QoS(10).reliable());
        if (estimate_topic != std::string{ground_truth_odometry_topic}) {
            // Best effort hears a reliable or a best-effort pose source.
            estimate_subscription_ =
                subscribe_pose(estimate_topic, rclcpp::QoS(10).best_effort());
        }
        error_subscription_ =
            create_subscription<racing_interfaces::msg::LocalizationError>(
                "/evaluation/localization_error", rclcpp::QoS(10).reliable(),
                [this](const racing_interfaces::msg::LocalizationError::
                           ConstSharedPtr &message) {
                    recorder_->record("/evaluation/localization_error",
                                      "racing_interfaces/msg/LocalizationError",
                                      message_timestamp(*message),
                                      localization_error_payload(*message));
                });
        flush_service_ = create_service<std_srvs::srv::Trigger>(
            "~/flush",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr &,
                const std_srvs::srv::Trigger::Response::SharedPtr &response) {
                flush(response);
            });
    }

   private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_pose(
        const std::string &topic, const rclcpp::QoS &qos) {
        return create_subscription<nav_msgs::msg::Odometry>(
            topic, qos,
            [this,
             topic](const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
                recorder_->record(topic, "nav_msgs/msg/Odometry",
                                  message_timestamp(*message),
                                  pose_payload(*message));
            });
    }

    void flush(const std_srvs::srv::Trigger::Response::SharedPtr &response) {
        try {
            recorder_->flush();
            response->success = true;
            response->message = recorder_->output_path().string();
        } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
        }
    }

    std::unique_ptr<EventRecorder> recorder_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        drive_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::TrackRelativeState>::SharedPtr
        state_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::SafetyStatus>::SharedPtr
        safety_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::ScenarioMetrics>::SharedPtr
        metrics_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        truth_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        estimate_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::LocalizationError>::SharedPtr
        error_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr flush_service_;
};

}  // namespace racing_recording

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<racing_recording::RecordingNode>());
    rclcpp::shutdown();
    return 0;
}
