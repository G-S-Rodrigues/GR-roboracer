#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <racing_interfaces/msg/safety_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "racing_common/track.hpp"
#include "racing_safety_supervisor/supervisor.hpp"

namespace racing_safety_supervisor {
namespace {

constexpr auto default_control_period = std::chrono::milliseconds{10};

double yaw_from_odometry(const nav_msgs::msg::Odometry &message) {
    const auto &orientation = message.pose.pose.orientation;
    const auto sin_yaw =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const auto cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y +
                                      orientation.z * orientation.z);
    return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

class SafetySupervisorNode : public rclcpp::Node {
   public:
    SafetySupervisorNode() : Node("racing_safety_supervisor") {
        SupervisorParams params;
        params.maximum_speed = declare_parameter("maximum_speed", 3.0);
        params.maximum_steering_angle =
            declare_parameter("maximum_steering_angle", 0.4);
        params.maximum_acceleration =
            declare_parameter("maximum_acceleration", 2.0);
        params.maximum_steering_rate =
            declare_parameter("maximum_steering_rate", 1.5);
        params.vehicle_half_width =
            declare_parameter("vehicle_half_width", 0.15);
        params.stale_input_timeout = std::chrono::milliseconds{
            declare_parameter<std::int64_t>("stale_input_timeout_ms", 100)};
        const auto track_path = declare_parameter(
            "track_path", std::string{"config/tracks/analytic_circle.yaml"});
        const auto period =
            std::chrono::milliseconds{declare_parameter<std::int64_t>(
                "control_period_ms", default_control_period.count())};
        if (period <= std::chrono::milliseconds::zero()) {
            throw std::invalid_argument("control_period_ms must be positive");
        }

        track_ = std::make_shared<racing_common::Track>(
            racing_common::Track::from_yaml(std::filesystem::path{track_path}));
        supervisor_ = std::make_unique<Supervisor>(params);
        state_.pose = track_->to_cartesian({0.0, 0.0, 0.0});

        command_publisher_ =
            create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", rclcpp::QoS(10).reliable());
        status_publisher_ =
            create_publisher<racing_interfaces::msg::SafetyStatus>(
                "/safety/status", rclcpp::QoS(10).reliable());
        command_subscription_ =
            create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                "/controller/drive", rclcpp::QoS(10).reliable(),
                [this](const ackermann_msgs::msg::AckermannDriveStamped::
                           ConstSharedPtr &message) {
                    command_.speed = message->drive.speed;
                    command_.steering_angle = message->drive.steering_angle;
                    command_.acceleration = message->drive.acceleration;
                    command_.steering_angle_velocity =
                        message->drive.steering_angle_velocity;
                    command_.received_at = Clock::now();
                    command_.has_command = true;
                });
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10).reliable(),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
                state_.pose = {message->pose.pose.position.x,
                               message->pose.pose.position.y,
                               yaw_from_odometry(*message)};
            });
        emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
            "~/emergency_stop",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr &,
                const std_srvs::srv::Trigger::Response::SharedPtr &response) {
                supervisor_->request_emergency_stop();
                response->success = true;
                response->message = "emergency stop latched";
            });
        clear_emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
            "~/clear_emergency_stop",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr &,
                const std_srvs::srv::Trigger::Response::SharedPtr &response) {
                supervisor_->clear_emergency_stop();
                command_.received_at = Clock::now();
                response->success = true;
                response->message = "emergency stop cleared";
            });
        timer_ =
            create_wall_timer(period, [this, period] { evaluate(period); });
    }

   private:
    void evaluate(std::chrono::milliseconds period) {
        const auto decision =
            supervisor_->evaluate(command_, state_, *track_, Clock::now());
        const auto stamp = get_clock()->now();

        ackermann_msgs::msg::AckermannDriveStamped command_message;
        command_message.header.stamp = stamp;
        command_message.header.frame_id = "base_link";
        command_message.drive.speed =
            static_cast<float>(decision.command.speed);
        command_message.drive.steering_angle =
            static_cast<float>(decision.command.steering_angle);
        command_message.drive.acceleration =
            static_cast<float>(decision.command.acceleration);
        command_message.drive.steering_angle_velocity =
            static_cast<float>(decision.command.steering_angle_velocity);
        command_publisher_->publish(command_message);

        racing_interfaces::msg::SafetyStatus status;
        status.header.stamp = stamp;
        status.header.frame_id = "base_link";
        status.source = get_name();
        status.valid_until =
            stamp + rclcpp::Duration::from_seconds(
                        static_cast<double>(period.count()) / 1000.0);
        status.active_clamps = decision.active_clamps;
        status.reason = static_cast<std::uint8_t>(decision.reason);
        status.heartbeat = heartbeat_++;
        status_publisher_->publish(status);
    }

    DriveCommand command_;
    VehicleState state_;
    std::shared_ptr<racing_common::Track> track_;
    std::unique_ptr<Supervisor> supervisor_;
    std::uint64_t heartbeat_{0};
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        command_publisher_;
    rclcpp::Publisher<racing_interfaces::msg::SafetyStatus>::SharedPtr
        status_publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        command_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        clear_emergency_stop_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racing_safety_supervisor

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<racing_safety_supervisor::SafetySupervisorNode>());
    rclcpp::shutdown();
    return 0;
}
