#include "racing_controller_baseline/pure_pursuit.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
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

// PP-1040: characterization of today's default outputs. The literals below
// were recorded from origin/main 3a847c79 (%.17g literals) before any
// production change, and this test is never edited: the default path must stay
// bit-identical.
struct Pp1040Case {
    VehicleState state;
    DriveCommand expected;
};

Trajectory pp1040_polygon() {
    Trajectory polygon;
    for (int k = 0; k < 12; ++k) {
        const double angle = k * M_PI / 6.0;
        polygon.push_back({3.0 * std::cos(angle), 3.0 * std::sin(angle)});
    }
    return polygon;
}

void expect_bit_equal(const DriveCommand &actual, const DriveCommand &expected,
                      std::size_t index) {
    EXPECT_EQ(actual.steering_angle, expected.steering_angle) << index;
    EXPECT_EQ(actual.speed, expected.speed) << index;
    EXPECT_EQ(actual.curvature, expected.curvature) << index;
    EXPECT_EQ(actual.target_x, expected.target_x) << index;
    EXPECT_EQ(actual.target_y, expected.target_y) << index;
}

TEST(PurePursuitTest, PP1040DefaultParametersReproduceTodaysOutputs) {
    const Trajectory polygon = pp1040_polygon();
    const auto &v0 = polygon[0];
    const auto &v1 = polygon[1];
    const auto &v11 = polygon[11];
    const double deg = M_PI / 180.0;
    const std::vector<Pp1040Case> closed_cases{
        // on-line (edge midpoint, heading along the edge)
        {{0.5 * (v0.x + v1.x), 0.5 * (v0.y + v1.y), 120.0 * deg},
         {-0.098733974167861727, 2.3073907718081674, -0.30016988741316475,
          2.4400075358434266, 1.6580686755098895}},
        // off-line, inside
        {{2.0, 0.5, 100.0 * deg},
         {-0.28018769116904874, 1.6025704645081524, -0.871992568464523,
          2.4645265886817556, 1.6335496226715605}},
        // off-line, outside
        {{3.5, 0.8, 110.0 * deg},
         {0.2742088059178277, 1.6195136633102003, 0.85240795923151325,
          2.5341418309524184, 1.5639343804008974}},
        // heading against the path
        {{0.5 * (v0.x + v1.x), 0.5 * (v0.y + v1.y), -60.0 * deg},
         {0.098733974167861963, 2.3073907718081657, 0.30016988741316547,
          2.4400075358434266, 1.6580686755098895}},
        // near the wrap: 97% of the way along the last edge
        {{v11.x + 0.97 * (v0.x - v11.x), v11.y + 0.97 * (v0.y - v11.y),
          75.0 * deg},
         {0.30826521729488116, 1.5267981359139136, 0.9648962946920645,
          2.7532386685568797, 0.92092582628906872}},
        // exactly on a vertex
        {{polygon[3].x, polygon[3].y, 180.0 * deg},
         {0.16918757026553233, 1.9767558677936727, 0.51763809020504181,
          -0.96592582628906798, 2.7411809548974793}},
    };
    ControllerParams closed;
    closed.closed_trajectory = true;
    closed.maximum_speed = 3.0;
    for (std::size_t i = 0; i < closed_cases.size(); ++i) {
        const auto actual =
            PurePursuit::compute(closed_cases[i].state, polygon, closed);
        expect_bit_equal(actual, closed_cases[i].expected, i);
    }

    const Trajectory open{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
    const std::vector<Pp1040Case> open_cases{
        {{-1.0, 0.2, 0.0},
         {-0.032661648068873367, 2.7297297297297294, -0.099009900990099015, 1,
          0}},  // before the start
        {{2.3, 2.5, 1.5},
         {0.47338956744176519, 1.1754404979442099, 1.5522346773374398, 2,
          2}},  // past the end
    };
    ControllerParams open_params;
    open_params.maximum_speed = 3.0;
    for (std::size_t i = 0; i < open_cases.size(); ++i) {
        const auto actual =
            PurePursuit::compute(open_cases[i].state, open, open_params);
        expect_bit_equal(actual, open_cases[i].expected, 100 + i);
    }
}

ControllerParams trajectory_speed_params(double minimum, double maximum) {
    ControllerParams params;
    params.speed_source = SpeedSource::TRAJECTORY;
    params.minimum_speed = minimum;
    params.maximum_speed = maximum;
    return params;
}

TEST(PurePursuitTest, PP1050TargetSpeedTakesNearestVertexByProgress) {
    const Trajectory line{{0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}, {2.0, 0.0, 3.0}};
    const auto params = trajectory_speed_params(0.0, 10.0);

    EXPECT_EQ(target_speed(line, 0.4, params), 1.0);
    EXPECT_EQ(target_speed(line, 0.6, params), 2.0);
    EXPECT_EQ(target_speed(line, 1.49, params), 2.0);
    EXPECT_EQ(target_speed(line, 2.0, params), 3.0);
}

TEST(PurePursuitTest, PP1060TargetSpeedClampsToTheSpeedLimits) {
    const Trajectory line{{0.0, 0.0, 0.0}, {1.0, 0.0, 10.0}, {2.0, 0.0, 1.7}};
    const auto params = trajectory_speed_params(0.5, 3.0);

    EXPECT_EQ(target_speed(line, 0.0, params), 0.5);
    EXPECT_EQ(target_speed(line, 1.0, params), 3.0);
    EXPECT_EQ(target_speed(line, 2.0, params), 1.7);
}

TEST(PurePursuitTest, PP1070TargetSpeedOnClosedTrajectories) {
    auto params = trajectory_speed_params(0.0, 10.0);
    params.closed_trajectory = true;

    // (a) no repeated closing vertex: the wrap segment v3 -> v0 spans 3..4.
    const Trajectory square{
        {0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}, {1.0, 1.0, 3.0}, {0.0, 1.0, 4.0}};
    EXPECT_EQ(target_speed(square, 3.9, params), 1.0);
    EXPECT_EQ(target_speed(square, 3.4, params), 4.0);

    // (b) closing vertex repeated with a different v: its zero-length
    // segment is skipped, and the repeat is the end of v3 -> v4.
    const Trajectory repeated{{0.0, 0.0, 1.0},
                              {1.0, 0.0, 2.0},
                              {1.0, 1.0, 3.0},
                              {0.0, 1.0, 4.0},
                              {0.0, 0.0, 9.0}};
    EXPECT_EQ(target_speed(repeated, 3.9, params), 9.0);
    EXPECT_EQ(target_speed(repeated, 3.4, params), 4.0);
}

TEST(PurePursuitTest, PP1080NonFiniteVelocityIsRejectedOnlyUnderTrajectory) {
    const double bad_values[] = {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()};
    const VehicleState state{0.5, 0.0, 0.0};
    for (const double bad : bad_values) {
        // The bad vertex is not the one the lookup lands on.
        const Trajectory line{
            {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {2.0, 0.0, bad}};
        const auto params = trajectory_speed_params(0.5, 3.0);
        EXPECT_THROW(target_speed(line, 0.2, params), std::invalid_argument);
        EXPECT_THROW(PurePursuit::compute(state, line, params),
                     std::invalid_argument);
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Trajectory nan_v{{0.0, 0.0, nan}, {1.0, 0.0, nan}, {2.0, 1.0, nan}};
    const Trajectory zero_v{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 1.0, 0.0}};
    const ControllerParams schedule;
    const auto with_nan = PurePursuit::compute(state, nan_v, schedule);
    const auto with_zero = PurePursuit::compute(state, zero_v, schedule);
    EXPECT_EQ(with_nan.steering_angle, with_zero.steering_angle);
    EXPECT_EQ(with_nan.speed, with_zero.speed);
    EXPECT_EQ(with_nan.curvature, with_zero.curvature);
    EXPECT_EQ(with_nan.target_x, with_zero.target_x);
    EXPECT_EQ(with_nan.target_y, with_zero.target_y);
}

}  // namespace
}  // namespace racing_controller_baseline
