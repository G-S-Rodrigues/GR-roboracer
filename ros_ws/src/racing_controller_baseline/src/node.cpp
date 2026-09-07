#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <racing_interfaces/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "racing_controller_baseline/pure_pursuit.hpp"

namespace racing_controller_baseline {

class PurePursuitNode : public rclcpp::Node {
   public:
    PurePursuitNode() : Node("racing_controller_baseline") {
        params_.lookahead_distance =
            declare_parameter("lookahead_distance", 1.0);
        params_.wheelbase = declare_parameter("wheelbase", 0.33);
        params_.minimum_speed = declare_parameter("minimum_speed", 0.5);
        params_.maximum_speed = declare_parameter("maximum_speed", 2.0);
        params_.curvature_speed_gain =
            declare_parameter("curvature_speed_gain", 1.0);
        params_.closed_trajectory =
            declare_parameter("closed_trajectory", true);

        command_publisher_ =
            create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/controller/drive", rclcpp::QoS(10).reliable());
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10).reliable(),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
                const auto &orientation = message->pose.pose.orientation;
                const auto sin_yaw = 2.0 * (orientation.w * orientation.z +
                                            orientation.x * orientation.y);
                const auto cos_yaw =
                    1.0 - 2.0 * (orientation.y * orientation.y +
                                 orientation.z * orientation.z);
                state_ = {message->pose.pose.position.x,
                          message->pose.pose.position.y,
                          std::atan2(sin_yaw, cos_yaw)};
                stamp_ = message->header.stamp;
                has_state_ = true;
                publish_if_ready();
            });
        trajectory_subscription_ =
            create_subscription<racing_interfaces::msg::Trajectory>(
                "/trajectory", rclcpp::QoS(1).reliable().transient_local(),
                [this](const racing_interfaces::msg::Trajectory::ConstSharedPtr
                           &message) {
                    trajectory_.clear();
                    trajectory_.reserve(message->points.size());
                    for (const auto &point : message->points) {
                        trajectory_.push_back({point.x, point.y});
                    }
                    publish_if_ready();
                });
    }

   private:
    void publish_if_ready() {
        if (!has_state_ || trajectory_.empty()) {
            return;
        }
        try {
            const auto output =
                PurePursuit::compute(state_, trajectory_, params_);
            ackermann_msgs::msg::AckermannDriveStamped message;
            message.header.stamp = stamp_;
            message.header.frame_id = "base_link";
            message.drive.steering_angle =
                static_cast<float>(output.steering_angle);
            message.drive.speed = static_cast<float>(output.speed);
            command_publisher_->publish(message);
        } catch (const std::invalid_argument &error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                                  error.what());
        }
    }

    ControllerParams params_;
    VehicleState state_;
    Trajectory trajectory_;
    builtin_interfaces::msg::Time stamp_;
    bool has_state_{false};
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        command_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    rclcpp::Subscription<racing_interfaces::msg::Trajectory>::SharedPtr
        trajectory_subscription_;
};

}  // namespace racing_controller_baseline

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<racing_controller_baseline::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
