#pragma once

#include <array>
#include <cstdint>

namespace racing_common {

enum class LongitudinalAction : std::uint8_t {
    ACCELERATION,
    VELOCITY,
};

enum class SteeringAction : std::uint8_t {
    STEERING_ANGLE,
    STEERING_VELOCITY,
};

struct DriveCommand {
    double steering_angle{};
    double steering_angle_velocity{};
    double speed{};
    double acceleration{};
};

struct EnvActionSpec {
    LongitudinalAction longitudinal;
    SteeringAction steering;
};

std::array<double, 2> apply_drive_command(const DriveCommand &command,
                                          const EnvActionSpec &spec);

}  // namespace racing_common
