#include "racing_common/action_map.hpp"

#include <gtest/gtest.h>

namespace {

constexpr racing_common::DriveCommand k_command{0.23, -0.71, 6.5, 1.25};

TEST(ActionMapTest, Common1060AccelerationSteeringAngle) {
    const auto action = racing_common::apply_drive_command(
        k_command, {racing_common::LongitudinalAction::ACCELERATION,
                    racing_common::SteeringAction::STEERING_ANGLE});

    EXPECT_DOUBLE_EQ(action[0], 0.23);
    EXPECT_DOUBLE_EQ(action[1], 1.25);
}

TEST(ActionMapTest, Common1060AccelerationSteeringVelocity) {
    const auto action = racing_common::apply_drive_command(
        k_command, {racing_common::LongitudinalAction::ACCELERATION,
                    racing_common::SteeringAction::STEERING_VELOCITY});

    EXPECT_DOUBLE_EQ(action[0], -0.71);
    EXPECT_DOUBLE_EQ(action[1], 1.25);
}

TEST(ActionMapTest, Common1060VelocitySteeringAngle) {
    const auto action = racing_common::apply_drive_command(
        k_command, {racing_common::LongitudinalAction::VELOCITY,
                    racing_common::SteeringAction::STEERING_ANGLE});

    EXPECT_DOUBLE_EQ(action[0], 0.23);
    EXPECT_DOUBLE_EQ(action[1], 6.5);
}

TEST(ActionMapTest, Common1060VelocitySteeringVelocity) {
    const auto action = racing_common::apply_drive_command(
        k_command, {racing_common::LongitudinalAction::VELOCITY,
                    racing_common::SteeringAction::STEERING_VELOCITY});

    EXPECT_DOUBLE_EQ(action[0], -0.71);
    EXPECT_DOUBLE_EQ(action[1], 6.5);
}

}  // namespace
