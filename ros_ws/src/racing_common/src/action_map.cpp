#include "racing_common/action_map.hpp"

namespace racing_common {

std::array<double, 2> apply_drive_command(const DriveCommand &command,
                                          const EnvActionSpec &spec) {
    const auto steering = spec.steering == SteeringAction::STEERING_ANGLE
                              ? command.steering_angle
                              : command.steering_angle_velocity;
    const auto longitudinal = spec.longitudinal == LongitudinalAction::VELOCITY
                                  ? command.speed
                                  : command.acceleration;
    return {steering, longitudinal};
}

}  // namespace racing_common
