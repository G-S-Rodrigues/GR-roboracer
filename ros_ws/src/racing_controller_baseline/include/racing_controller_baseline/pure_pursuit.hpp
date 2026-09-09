#pragma once

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
};

using Trajectory = std::vector<TrajectoryPoint>;

struct ControllerParams {
    double lookahead_distance{1.0};
    double wheelbase{0.33};
    double minimum_speed{0.5};
    double maximum_speed{2.0};
    double curvature_speed_gain{1.0};
    bool closed_trajectory{false};
};

struct DriveCommand {
    double steering_angle{};
    double speed{};
    double curvature{};
    double target_x{};
    double target_y{};
};

double curvature_speed_schedule(double kappa, const ControllerParams &params);

class PurePursuit {
   public:
    static DriveCommand compute(const VehicleState &state,
                                const Trajectory &trajectory,
                                const ControllerParams &params);
};

}  // namespace racing_controller_baseline
