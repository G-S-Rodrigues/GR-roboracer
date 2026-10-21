#include "racing_controller_baseline/pure_pursuit.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
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

TEST(PurePursuitTest, PP1090LookaheadScalesWithMeasuredSpeedAboveTheFloor) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    ControllerParams fixed;
    fixed.lookahead_distance = 1.0;
    fixed.lookahead_gain = 0.0;
    for (const double speed : {0.0, 2.5, 100.0, -1.0, nan, inf}) {
        EXPECT_EQ(lookahead_distance(speed, fixed), 1.0) << speed;
    }

    ControllerParams scaled;
    scaled.lookahead_distance = 1.0;
    scaled.lookahead_gain = 0.5;
    EXPECT_EQ(lookahead_distance(1.0, scaled), 1.0);
    EXPECT_EQ(lookahead_distance(4.0, scaled), 2.0);
    EXPECT_EQ(lookahead_distance(-3.0, scaled), 1.0);
    EXPECT_THROW(lookahead_distance(nan, scaled), std::invalid_argument);
    EXPECT_THROW(lookahead_distance(inf, scaled), std::invalid_argument);
}

TEST(PurePursuitTest, PP1100LookaheadGainMustBeFiniteAndNonNegative) {
    const Trajectory line{{0.0, 0.0, 1.0}, {2.0, 0.0, 1.0}};
    const VehicleState state{};
    for (const double bad : {-0.1, std::numeric_limits<double>::quiet_NaN()}) {
        ControllerParams params;
        params.lookahead_gain = bad;
        EXPECT_THROW(validate_parameters(params), std::invalid_argument);
        EXPECT_THROW(PurePursuit::compute(state, line, params),
                     std::invalid_argument);
    }

    for (const auto source :
         {SpeedSource::CURVATURE_SCHEDULE, SpeedSource::TRAJECTORY}) {
        ControllerParams params;
        params.lookahead_gain = 0.5;
        params.speed_source = source;
        EXPECT_NO_THROW(validate_parameters(params));
        EXPECT_NO_THROW(PurePursuit::compute(state, line, params));
    }
}

// A closed 12-gon of radius 3 with v cycling 0.7 / 1.9 / 2.6 by vertex.
Trajectory pp1110_polygon(bool with_v) {
    const double cycle[] = {0.7, 1.9, 2.6};
    Trajectory polygon;
    for (int k = 0; k < 12; ++k) {
        const double angle = k * M_PI / 6.0;
        polygon.push_back({3.0 * std::cos(angle), 3.0 * std::sin(angle),
                           with_v ? cycle[k % 3] : 0.0});
    }
    return polygon;
}

void expect_commands_equal(const DriveCommand &actual,
                           const DriveCommand &expected) {
    EXPECT_EQ(actual.steering_angle, expected.steering_angle);
    EXPECT_EQ(actual.speed, expected.speed);
    EXPECT_EQ(actual.curvature, expected.curvature);
    EXPECT_EQ(actual.target_x, expected.target_x);
    EXPECT_EQ(actual.target_y, expected.target_y);
}

TEST(PurePursuitTest, PP1110TrajectorySpeedIsTheClampedVAtTheClosestPoint) {
    const Trajectory with_v = pp1110_polygon(true);
    const Trajectory without_v = pp1110_polygon(false);
    const double cycle[] = {0.7, 1.9, 2.6};
    struct Probe {
        std::size_t edge;
        double ratio;
        double inward;
        std::size_t vertex;  // the vertex the test names for this state
    };
    // Projection ratios stay well away from 0.5.
    const Probe probes[] = {{1, 0.2, 0.0, 1},  {4, 0.8, 0.0, 5},
                            {7, 0.3, 0.0, 7},  {11, 0.9, 0.0, 0},
                            {2, 0.2, 0.15, 2}, {8, 0.7, 0.0, 9}};
    ControllerParams schedule;
    schedule.minimum_speed = 0.5;
    schedule.maximum_speed = 3.0;
    ControllerParams trajectory = schedule;
    trajectory.speed_source = SpeedSource::TRAJECTORY;
    schedule.closed_trajectory = true;
    trajectory.closed_trajectory = true;

    for (const auto &probe : probes) {
        const auto &a = with_v[probe.edge];
        const auto &b = with_v[(probe.edge + 1) % 12];
        const double x = a.x + probe.ratio * (b.x - a.x);
        const double y = a.y + probe.ratio * (b.y - a.y);
        const double norm = std::hypot(x, y);
        const VehicleState state{x - probe.inward * x / norm,
                                 y - probe.inward * y / norm,
                                 std::atan2(b.y - a.y, b.x - a.x)};

        // (a) the default source ignores v.
        expect_commands_equal(PurePursuit::compute(state, with_v, schedule),
                              PurePursuit::compute(state, without_v, schedule));

        // (b) TRAJECTORY changes the speed only.
        const auto by_schedule = PurePursuit::compute(state, with_v, schedule);
        const auto by_trajectory =
            PurePursuit::compute(state, with_v, trajectory);
        EXPECT_EQ(by_trajectory.speed, cycle[probe.vertex % 3]);
        EXPECT_EQ(by_trajectory.steering_angle, by_schedule.steering_angle);
        EXPECT_EQ(by_trajectory.target_x, by_schedule.target_x);
        EXPECT_EQ(by_trajectory.target_y, by_schedule.target_y);
    }
}

TEST(PurePursuitTest, PP1120LookaheadUsesMeasuredSpeedNotTrajectoryV) {
    Trajectory line;
    for (int k = 0; k <= 5; ++k) {
        line.push_back({static_cast<double>(k), 0.0, 1.0});
    }
    ControllerParams params;
    params.lookahead_distance = 1.0;
    params.lookahead_gain = 0.5;
    params.minimum_speed = 0.5;
    params.maximum_speed = 5.0;
    const VehicleState state{0.0, 0.0, 0.0, 4.0};

    // (a) schedule source, v = 0 everywhere: 0.5 * 4.0 = 2.0 m ahead.
    Trajectory zero_v = line;
    for (auto &point : zero_v) {
        point.v = 0.0;
    }
    EXPECT_EQ(PurePursuit::compute(state, zero_v, params).target_x, 2.0);

    // (b) trajectory source, v = 1.0: gain * v would give the 1.0 floor.
    params.speed_source = SpeedSource::TRAJECTORY;
    EXPECT_EQ(PurePursuit::compute(state, line, params).target_x, 2.0);
}

TEST(PurePursuitTest, PP1130TrajectorySpeedFollowsTheCarAcrossAProfileStep) {
    Trajectory line;
    for (int k = 0; k <= 10; ++k) {
        line.push_back({static_cast<double>(k), 0.0, k < 5 ? 1.0 : 3.0});
    }
    ControllerParams params;
    params.speed_source = SpeedSource::TRAJECTORY;
    params.minimum_speed = 0.5;
    params.maximum_speed = 3.0;

    EXPECT_EQ(PurePursuit::compute({2.0, 0.0, 0.0}, line, params).speed, 1.0);
    EXPECT_EQ(PurePursuit::compute({8.0, 0.0, 0.0}, line, params).speed, 3.0);
}

TEST(PurePursuitTest, PP1150DefaultParametersIgnoreTheMeasuredSpeed) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Trajectory polygon = pp1040_polygon();
    const double deg = M_PI / 180.0;
    const auto &v0 = polygon[0];
    const auto &v1 = polygon[1];
    const auto &v11 = polygon[11];
    // The same states as PP-1040, speed left to the loop below.
    const VehicleState states[] = {
        {0.5 * (v0.x + v1.x), 0.5 * (v0.y + v1.y), 120.0 * deg},
        {2.0, 0.5, 100.0 * deg},
        {3.5, 0.8, 110.0 * deg},
        {0.5 * (v0.x + v1.x), 0.5 * (v0.y + v1.y), -60.0 * deg},
        {v11.x + 0.97 * (v0.x - v11.x), v11.y + 0.97 * (v0.y - v11.y),
         75.0 * deg},
        {polygon[3].x, polygon[3].y, 180.0 * deg},
    };
    ControllerParams params;
    params.closed_trajectory = true;
    params.maximum_speed = 3.0;
    for (const auto &state : states) {
        const auto reference = PurePursuit::compute(state, polygon, params);
        for (const double speed : {2.7, -1.0, nan, inf}) {
            VehicleState measured = state;
            measured.speed = speed;
            expect_commands_equal(
                PurePursuit::compute(measured, polygon, params), reference);
        }
    }
}

TEST(PurePursuitTest, PP1140ParseSpeedSourceAcceptsOnlyTheTwoNames) {
    EXPECT_EQ(parse_speed_source("curvature_schedule"),
              SpeedSource::CURVATURE_SCHEDULE);
    EXPECT_EQ(parse_speed_source("trajectory"), SpeedSource::TRAJECTORY);
    for (const std::string bad : {"", "Trajectory", "track"}) {
        try {
            parse_speed_source(bad);
            ADD_FAILURE() << "no throw for '" << bad << "'";
        } catch (const std::invalid_argument &error) {
            const std::string message = error.what();
            EXPECT_NE(message.find("curvature_schedule"), std::string::npos);
            EXPECT_NE(message.find("trajectory"), std::string::npos);
        }
    }
}

}  // namespace
}  // namespace racing_controller_baseline
