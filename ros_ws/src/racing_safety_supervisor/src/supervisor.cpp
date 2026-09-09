#include "racing_safety_supervisor/supervisor.hpp"

#include <algorithm>
#include <stdexcept>

namespace racing_safety_supervisor {
namespace {

std::uint32_t clamp_mask(Clamp clamp) {
    return static_cast<std::uint32_t>(clamp);
}

DriveCommand stopped_command(const DriveCommand &command) {
    auto stopped = command;
    stopped.speed = 0.0;
    stopped.steering_angle = 0.0;
    stopped.acceleration = 0.0;
    stopped.steering_angle_velocity = 0.0;
    return stopped;
}

}  // namespace

Supervisor::Supervisor(SupervisorParams params) : params_(params) {
    if (params_.maximum_speed < 0.0 || params_.maximum_steering_angle < 0.0 ||
        params_.maximum_acceleration < 0.0 ||
        params_.maximum_steering_rate < 0.0 ||
        params_.vehicle_half_width < 0.0 ||
        params_.stale_input_timeout < std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("safety limits must be non-negative");
    }
}

SafetyDecision Supervisor::evaluate(const DriveCommand &command,
                                    const VehicleState &state,
                                    const racing_common::Track &track,
                                    Clock::time_point now) {
    SafetyDecision decision{command};
    if (command.has_command &&
        now - command.received_at > params_.stale_input_timeout) {
        emergency_stop_latched_ = true;
        decision.command = stopped_command(command);
        decision.active_clamps =
            clamp_mask(Clamp::STALE_INPUT) | clamp_mask(Clamp::ESTOP);
        decision.reason = SafetyReason::STALE_INPUT;
        return decision;
    }

    if (emergency_stop_latched_) {
        decision.command = stopped_command(command);
        decision.active_clamps = clamp_mask(Clamp::ESTOP);
        decision.reason = SafetyReason::ESTOP_LATCHED;
        return decision;
    }

    const auto frenet = track.to_frenet(state.pose);
    if (!track.is_inside(frenet, params_.vehicle_half_width)) {
        decision.command = stopped_command(command);
        decision.active_clamps = clamp_mask(Clamp::TRACK_LIMIT);
        decision.reason = SafetyReason::TRACK_LIMIT;
        return decision;
    }

    const auto clamp_field = [&decision](double &field, double limit,
                                         Clamp clamp) {
        const auto clamped = std::clamp(field, -limit, limit);
        if (clamped != field) {
            field = clamped;
            decision.active_clamps |= clamp_mask(clamp);
        }
    };

    clamp_field(decision.command.speed, params_.maximum_speed, Clamp::SPEED);
    clamp_field(decision.command.steering_angle, params_.maximum_steering_angle,
                Clamp::STEERING);
    clamp_field(decision.command.acceleration, params_.maximum_acceleration,
                Clamp::ACCELERATION);
    clamp_field(decision.command.steering_angle_velocity,
                params_.maximum_steering_rate, Clamp::STEERING_RATE);
    if (decision.active_clamps != 0U) {
        decision.reason = SafetyReason::COMMAND_LIMIT;
    }
    return decision;
}

void Supervisor::request_emergency_stop() { emergency_stop_latched_ = true; }

void Supervisor::clear_emergency_stop() { emergency_stop_latched_ = false; }

bool Supervisor::emergency_stop_latched() const {
    return emergency_stop_latched_;
}

}  // namespace racing_safety_supervisor
