#ifndef RACING_SIM_ADAPTER__CONTRACT_HPP_
#define RACING_SIM_ADAPTER__CONTRACT_HPP_

#include <cstddef>
#include <string_view>

namespace racing_sim_adapter::contract {

inline constexpr std::string_view kNodeName{"racing_sim"};

// /scan and /odom carry their standard ROS meanings: noisy ranges, and noisy
// dead reckoning in kOdomFrame. The exact values are published alongside on
// /ground_truth/*, for evaluation, metrics and the supervisor only.
inline constexpr std::string_view kScanTopic{"/scan"};
inline constexpr std::string_view kOdometryTopic{"/odom"};
inline constexpr std::string_view kGroundTruthScanTopic{"/ground_truth/scan"};
inline constexpr std::string_view kGroundTruthOdometryTopic{
    "/ground_truth/odom"};
inline constexpr std::string_view kImuTopic{"/imu"};
inline constexpr std::string_view kTrackRelativeStateTopic{
    "/ground_truth/track_relative_state"};
inline constexpr std::string_view kDriveTopic{"/drive"};

inline constexpr std::string_view kResetService{"/racing_sim/reset"};
inline constexpr std::string_view kStepModeService{"/racing_sim/step_mode"};

// What a backend can do. Each is a read-only parameter on kNodeName that the
// backend declares itself; conformance tests select by them, and a test whose
// capability is absent is reported not applicable by name.
inline constexpr std::string_view kCapabilityClockParameter{"capability.clock"};
inline constexpr std::string_view kClockOwnsClock{"owns_clock"};      // /clock
inline constexpr std::string_view kClockFreeRunning{"free_running"};  // stamps
inline constexpr std::string_view kCapabilityResetParameter{"capability.reset"};
inline constexpr std::string_view kResetSeededPose{"seeded_pose"};
inline constexpr std::string_view kResetInitialPoseOnly{"initial_pose_only"};
// The next three are booleans.
inline constexpr std::string_view kCapabilityStepModeParameter{
    "capability.step_mode"};
inline constexpr std::string_view kCapabilityTimeScaleParameter{
    "capability.time_scale"};
inline constexpr std::string_view kCapabilityStartHeldParameter{
    "capability.start_held"};

inline constexpr std::string_view kMapFrame{"map"};
inline constexpr std::string_view kOdomFrame{"odom"};
inline constexpr std::string_view kBaseFrame{"base_link"};
inline constexpr std::string_view kLaserFrame{"lidar"};

inline constexpr double kDefaultPublishRateHz{100.0};
inline constexpr std::size_t kSensorQueueDepth{5};
inline constexpr std::size_t kStateQueueDepth{10};

}  // namespace racing_sim_adapter::contract

#endif  // RACING_SIM_ADAPTER__CONTRACT_HPP_
