#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace racing_controller_baseline {

struct VehicleState {
    double x{};
    double y{};
    double yaw{};
};

struct TrajectoryPoint {
    double x{};
    double y{};
    double v{};
};

using Trajectory = std::vector<TrajectoryPoint>;

enum class SpeedSource : std::uint8_t { CURVATURE_SCHEDULE, TRAJECTORY };

struct ControllerParams {
    double lookahead_distance{1.0};
    double wheelbase{0.33};
    double minimum_speed{0.5};
    double maximum_speed{2.0};
    double curvature_speed_gain{1.0};
    bool closed_trajectory{false};
    SpeedSource speed_source{SpeedSource::CURVATURE_SCHEDULE};
    double lookahead_gain{0.0};
};

struct DriveCommand {
    double steering_angle{};
    double speed{};
    double curvature{};
    double target_x{};
    double target_y{};
};

double curvature_speed_schedule(double kappa, const ControllerParams &params);

// Speed of the trajectory vertex nearest `progress` by arc length, clamped to
// [minimum_speed, maximum_speed].
double target_speed(const Trajectory &trajectory, double progress,
                    const ControllerParams &params);

class PurePursuit {
   public:
    static DriveCommand compute(const VehicleState &state,
                                const Trajectory &trajectory,
                                const ControllerParams &params);
};

}  // namespace racing_controller_baseline
