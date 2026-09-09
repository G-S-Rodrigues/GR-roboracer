#pragma once

#include <chrono>
#include <cstdint>

#include "racing_common/track.hpp"

namespace racing_safety_supervisor {

using Clock = std::chrono::steady_clock;

enum class Clamp : std::uint8_t {
    NONE = 0,
    TRACK_LIMIT = 1,
    SPEED = 2,
    STEERING = 4,
    ACCELERATION = 8,
    STEERING_RATE = 16,
    STALE_INPUT = 32,
    ESTOP = 64,
};

enum class SafetyReason : std::uint8_t {
    NONE = 0,
    TRACK_LIMIT = 1,
    COMMAND_LIMIT = 2,
    STALE_INPUT = 3,
    ESTOP_LATCHED = 4,
};

struct DriveCommand {
    double speed{};
    double steering_angle{};
    double acceleration{};
    double steering_angle_velocity{};
    Clock::time_point received_at{};
    // False until the first real command arrives. Distinguishes "never heard
    // from the controller yet" (safe to just command zero) from "the command
    // stream went stale after flowing normally" (a genuine anomaly that
    // latches an emergency stop). Without this, a supervisor whose ROS shell
    // starts before the controller's first message reaches it — an ordinary
    // ROS 2 discovery race on every multi-process launch — would latch a
    // permanent, unrecoverable emergency stop before ever being commanded.
    bool has_command{false};
};

struct VehicleState {
    racing_common::CartesianPose pose;
};

struct SupervisorParams {
    double maximum_speed{3.0};
    double maximum_steering_angle{0.4};
    double maximum_acceleration{2.0};
    double maximum_steering_rate{1.5};
    double vehicle_half_width{0.15};
    std::chrono::milliseconds stale_input_timeout{100};
};

struct SafetyDecision {
    DriveCommand command;
    std::uint32_t active_clamps{};
    SafetyReason reason{SafetyReason::NONE};
};

class Supervisor {
   public:
    explicit Supervisor(SupervisorParams params);

    SafetyDecision evaluate(const DriveCommand &command,
                            const VehicleState &state,
                            const racing_common::Track &track,
                            Clock::time_point now);
    void request_emergency_stop();
    void clear_emergency_stop();
    bool emergency_stop_latched() const;

   private:
    SupervisorParams params_;
    bool emergency_stop_latched_{false};
};

}  // namespace racing_safety_supervisor
