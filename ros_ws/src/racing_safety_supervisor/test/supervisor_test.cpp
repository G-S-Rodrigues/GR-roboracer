#include "racing_safety_supervisor/supervisor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

namespace {

using racing_safety_supervisor::Clamp;
using racing_safety_supervisor::DriveCommand;
using racing_safety_supervisor::SafetyReason;
using racing_safety_supervisor::Supervisor;
using racing_safety_supervisor::SupervisorParams;
using racing_safety_supervisor::VehicleState;

constexpr auto now =
    std::chrono::steady_clock::time_point{std::chrono::seconds{10}};

racing_common::Track test_track() {
    return racing_common::Track::from_yaml(
        std::filesystem::path{RACING_SAFETY_TEST_TRACK});
}

SupervisorParams params() {
    SupervisorParams result;
    result.maximum_speed = 3.0;
    result.maximum_steering_angle = 0.4;
    result.maximum_acceleration = 2.0;
    result.maximum_steering_rate = 1.5;
    result.vehicle_half_width = 0.15;
    result.stale_input_timeout = std::chrono::milliseconds{100};
    return result;
}

DriveCommand nominal_command() { return DriveCommand{1.0, 0.1, 0.5, 0.2, now}; }

VehicleState nominal_state() { return VehicleState{{10.0, 0.0, 0.0}}; }

TEST(SupervisorTest, Safe1010EachConstraintClampsIndependently) {
    const auto track = test_track();

    struct Case {
        DriveCommand command;
        VehicleState state;
        Clamp expected_clamp;
        double expected_speed;
        double expected_steering;
        double expected_acceleration;
        double expected_rate;
        SafetyReason expected_reason;
    };

    auto speed = nominal_command();
    speed.speed = 4.0;
    auto steering = nominal_command();
    steering.steering_angle = -0.8;
    auto acceleration = nominal_command();
    acceleration.acceleration = -3.0;
    auto rate = nominal_command();
    rate.steering_angle_velocity = 2.0;
    auto outside = nominal_state();
    outside.pose = track.to_cartesian({0.0, 2.9, 0.0});

    const Case cases[] = {
        {speed, nominal_state(), Clamp::SPEED, 3.0, 0.1, 0.5, 0.2,
         SafetyReason::COMMAND_LIMIT},
        {steering, nominal_state(), Clamp::STEERING, 1.0, -0.4, 0.5, 0.2,
         SafetyReason::COMMAND_LIMIT},
        {acceleration, nominal_state(), Clamp::ACCELERATION, 1.0, 0.1, -2.0,
         0.2, SafetyReason::COMMAND_LIMIT},
        {rate, nominal_state(), Clamp::STEERING_RATE, 1.0, 0.1, 0.5, 1.5,
         SafetyReason::COMMAND_LIMIT},
        {nominal_command(), outside, Clamp::TRACK_LIMIT, 0.0, 0.0, 0.0, 0.0,
         SafetyReason::TRACK_LIMIT},
    };

    for (const auto &test_case : cases) {
        Supervisor supervisor{params()};
        const auto decision =
            supervisor.evaluate(test_case.command, test_case.state, track, now);

        EXPECT_EQ(decision.active_clamps,
                  static_cast<std::uint32_t>(test_case.expected_clamp));
        EXPECT_EQ(decision.reason, test_case.expected_reason);
        EXPECT_DOUBLE_EQ(decision.command.speed, test_case.expected_speed);
        EXPECT_DOUBLE_EQ(decision.command.steering_angle,
                         test_case.expected_steering);
        EXPECT_DOUBLE_EQ(decision.command.acceleration,
                         test_case.expected_acceleration);
        EXPECT_DOUBLE_EQ(decision.command.steering_angle_velocity,
                         test_case.expected_rate);
    }
}

TEST(SupervisorTest, Safe1020StaleInputTriggersEmergencyStop) {
    auto command = nominal_command();
    command.received_at = now - std::chrono::milliseconds{101};
    Supervisor supervisor{params()};

    const auto decision =
        supervisor.evaluate(command, nominal_state(), test_track(), now);

    EXPECT_EQ(decision.active_clamps,
              static_cast<std::uint32_t>(Clamp::STALE_INPUT) |
                  static_cast<std::uint32_t>(Clamp::ESTOP));
    EXPECT_EQ(decision.reason, SafetyReason::STALE_INPUT);
    EXPECT_DOUBLE_EQ(decision.command.speed, 0.0);
    EXPECT_DOUBLE_EQ(decision.command.steering_angle, 0.0);
    EXPECT_DOUBLE_EQ(decision.command.acceleration, 0.0);
    EXPECT_DOUBLE_EQ(decision.command.steering_angle_velocity, 0.0);
    EXPECT_TRUE(supervisor.emergency_stop_latched());
}

TEST(SupervisorTest, Safe1030EmergencyStopStaysLatchedUntilExplicitClear) {
    Supervisor supervisor{params()};
    supervisor.request_emergency_stop();

    const auto latched = supervisor.evaluate(nominal_command(), nominal_state(),
                                             test_track(), now);

    EXPECT_EQ(latched.active_clamps, static_cast<std::uint32_t>(Clamp::ESTOP));
    EXPECT_EQ(latched.reason, SafetyReason::ESTOP_LATCHED);
    EXPECT_DOUBLE_EQ(latched.command.speed, 0.0);
    EXPECT_TRUE(supervisor.emergency_stop_latched());

    supervisor.clear_emergency_stop();
    const auto cleared = supervisor.evaluate(nominal_command(), nominal_state(),
                                             test_track(), now);

    EXPECT_EQ(cleared.active_clamps, static_cast<std::uint32_t>(Clamp::NONE));
    EXPECT_EQ(cleared.reason, SafetyReason::NONE);
    EXPECT_DOUBLE_EQ(cleared.command.speed, nominal_command().speed);
    EXPECT_FALSE(supervisor.emergency_stop_latched());
}

}  // namespace
