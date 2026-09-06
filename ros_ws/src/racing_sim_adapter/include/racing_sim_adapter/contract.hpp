#ifndef RACING_SIM_ADAPTER__CONTRACT_HPP_
#define RACING_SIM_ADAPTER__CONTRACT_HPP_

#include <cstddef>
#include <string_view>

namespace racing_sim_adapter::contract {

inline constexpr std::string_view kNodeName{"racing_sim"};

inline constexpr std::string_view kScanTopic{"/scan"};
inline constexpr std::string_view kOdometryTopic{"/odom"};
inline constexpr std::string_view kImuTopic{"/imu"};
inline constexpr std::string_view kTrackRelativeStateTopic{
    "/ground_truth/track_relative_state"};
inline constexpr std::string_view kDriveTopic{"/drive"};

inline constexpr std::string_view kResetService{"/racing_sim/reset"};
inline constexpr std::string_view kStepModeService{"/racing_sim/step_mode"};

inline constexpr std::string_view kMapFrame{"map"};
inline constexpr std::string_view kBaseFrame{"base_link"};
inline constexpr std::string_view kLaserFrame{"laser"};

inline constexpr double kDefaultPublishRateHz{100.0};
inline constexpr std::size_t kSensorQueueDepth{5};
inline constexpr std::size_t kStateQueueDepth{10};

}  // namespace racing_sim_adapter::contract

#endif  // RACING_SIM_ADAPTER__CONTRACT_HPP_
