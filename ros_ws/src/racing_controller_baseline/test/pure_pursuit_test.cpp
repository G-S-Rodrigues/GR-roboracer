#include "racing_controller_baseline/pure_pursuit.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace racing_controller_baseline {
namespace {

constexpr double k_tolerance = 1.0e-9;

TEST(PurePursuitTest, PP1010MatchesHandComputedGeometry) {
    const VehicleState state{0.0, 0.0, 0.0};
    const Trajectory trajectory{{0.0, 0.0}, {2.0, 2.0}};
    ControllerParams params;
    params.lookahead_distance = std::sqrt(8.0);
    params.wheelbase = 0.5;
    params.minimum_speed = 1.0;
    params.maximum_speed = 4.0;
    params.curvature_speed_gain = 2.0;

    const auto command = PurePursuit::compute(state, trajectory, params);

    const double expected_curvature = 0.5;
    EXPECT_NEAR(command.curvature, expected_curvature, k_tolerance);
    EXPECT_NEAR(command.steering_angle,
                std::atan(params.wheelbase * expected_curvature), k_tolerance);
    EXPECT_NEAR(command.speed, 2.0, k_tolerance);
}

TEST(PurePursuitTest, PP1020LookaheadClampsAtOpenPathEnds) {
    const Trajectory trajectory{{0.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}};
    ControllerParams params;
    params.lookahead_distance = 1.0;
    params.wheelbase = 0.5;

    const auto before_start =
        PurePursuit::compute({-2.0, 0.0, 0.0}, trajectory, params);
    EXPECT_NEAR(before_start.target_x, 1.0, k_tolerance);
    EXPECT_NEAR(before_start.target_y, 0.0, k_tolerance);

    const auto after_end =
        PurePursuit::compute({3.0, 1.0, 0.0}, trajectory, params);
    EXPECT_NEAR(after_end.target_x, 2.0, k_tolerance);
    EXPECT_NEAR(after_end.target_y, 1.0, k_tolerance);
}

TEST(PurePursuitTest, PP1030SpeedScheduleIsMonotonicInAbsoluteCurvature) {
    ControllerParams params;
    params.minimum_speed = 1.0;
    params.maximum_speed = 5.0;
    params.curvature_speed_gain = 3.0;

    const auto straight = curvature_speed_schedule(0.0, params);
    const auto gentle_left = curvature_speed_schedule(0.2, params);
    const auto gentle_right = curvature_speed_schedule(-0.2, params);
    const auto sharp = curvature_speed_schedule(1.0, params);

    EXPECT_DOUBLE_EQ(straight, params.maximum_speed);
    EXPECT_DOUBLE_EQ(gentle_left, gentle_right);
    EXPECT_GT(straight, gentle_left);
    EXPECT_GT(gentle_left, sharp);
    EXPECT_GE(sharp, params.minimum_speed);
}

}  // namespace
}  // namespace racing_controller_baseline
