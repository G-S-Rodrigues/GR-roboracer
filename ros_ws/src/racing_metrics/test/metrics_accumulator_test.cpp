#include "racing_metrics/metrics_accumulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace racing_metrics {
namespace {

// METRICS-1010
TEST(MetricsAccumulatorTest, AccumulatesHandComputedSyntheticRun) {
    const RunProvenance provenance{"metrics_test",
                                   "synthetic",
                                   42,
                                   4,
                                   "revision-abc",
                                   "sha256:image",
                                   "track-version-1",
                                   "vehicle-version-2"};
    MetricsAccumulator accumulator{provenance, 10.0};

    accumulator.observe({1.0, 8.0, 0.50, 0.10, false, false});
    accumulator.observe({2.0, 9.0, 0.40, 0.40, true, true});
    accumulator.observe({3.0, 0.5, 0.30, 0.20, true, true});
    accumulator.observe({4.0, 4.0, 0.20, 0.30, false, false});
    accumulator.observe({5.0, 8.1, 0.25, 0.00, true, true});

    const auto result = accumulator.summary();
    EXPECT_EQ(result.source, "metrics_test");
    EXPECT_EQ(result.scenario_id, "synthetic");
    EXPECT_EQ(result.seed, 42U);
    EXPECT_EQ(result.timestep_ratio, 4U);
    EXPECT_EQ(result.code_revision, "revision-abc");
    EXPECT_EQ(result.container_image_digest, "sha256:image");
    EXPECT_EQ(result.track_version, "track-version-1");
    EXPECT_EQ(result.vehicle_parameter_version, "vehicle-version-2");

    EXPECT_TRUE(result.lap_completed);
    EXPECT_DOUBLE_EQ(result.lap_time, 4.0);
    EXPECT_EQ(result.collision_count, 2U);
    EXPECT_DOUBLE_EQ(result.minimum_wall_clearance, 0.20);
    EXPECT_DOUBLE_EQ(result.maximum_tracking_error, 0.40);
    EXPECT_NEAR(result.p95_tracking_error, 0.38, 1e-12);
    EXPECT_EQ(result.control_saturation_events, 2U);

    const auto json = to_json(result, 5123456789, "map");
    EXPECT_NE(json.find("\"header\":{\"stamp\":{\"sec\":5,"
                        "\"nanosec\":123456789},\"frame_id\":\"map\"}"),
              std::string::npos);
    EXPECT_NE(json.find("\"scenario_id\":\"synthetic\""), std::string::npos);
    EXPECT_NE(json.find("\"seed\":42"), std::string::npos);
    EXPECT_NE(json.find("\"timestep_ratio\":4"), std::string::npos);
    EXPECT_NE(json.find("\"code_revision\":\"revision-abc\""),
              std::string::npos);
    EXPECT_NE(json.find("\"container_image_digest\":\"sha256:image\""),
              std::string::npos);
    EXPECT_NE(json.find("\"track_version\":\"track-version-1\""),
              std::string::npos);
    EXPECT_NE(json.find("\"vehicle_parameter_version\":"
                        "\"vehicle-version-2\""),
              std::string::npos);
}

// METRICS-1020: a sim held at t=0 (or in STEPPED mode) republishes the same
// simulated instant; counting each copy would weight the percentile by how
// long the hold happened to last.
TEST(MetricsAccumulatorTest, RepublishedInstantIsOneSample) {
    const RunProvenance provenance{"source",   "scenario", 1,       1,
                                   "revision", "digest",   "track", "vehicle"};
    MetricsAccumulator once{provenance, 10.0};
    MetricsAccumulator repeated{provenance, 10.0};

    once.observe({0.0, 0.0, 1.0, 0.9, false, false});
    for (int copy = 0; copy < 50; ++copy) {
        repeated.observe({0.0, 0.0, 1.0, 0.9, false, false});
    }
    for (int tick = 1; tick <= 19; ++tick) {
        const auto time = static_cast<double>(tick);
        once.observe({time, time * 0.1, 1.0, 0.1, false, false});
        repeated.observe({time, time * 0.1, 1.0, 0.1, false, false});
    }

    EXPECT_DOUBLE_EQ(repeated.summary().p95_tracking_error,
                     once.summary().p95_tracking_error);
}

TEST(MetricsAccumulatorTest, RejectsInvalidRunInput) {
    const RunProvenance provenance{"source",   "scenario", 1,       1,
                                   "revision", "digest",   "track", "vehicle"};
    EXPECT_THROW((MetricsAccumulator{provenance, 0.0}), std::invalid_argument);

    MetricsAccumulator accumulator{provenance, 10.0};
    EXPECT_THROW(accumulator.observe({-1.0, 0.0, 1.0, 0.0, false, false}),
                 std::invalid_argument);
}

RunProvenance lap_provenance() {
    return {"source",   "scenario", 1,       1,
            "revision", "digest",   "track", "vehicle"};
}

// METRICS-1060: three laps at a constant 2 m/s on a 10 m track, sampled
// every 0.5 s. Each lap is 5 s and the run is complete only at the third.
TEST(MetricsAccumulatorTest, CountsEveryLapAndCompletesOnTheLast) {
    MetricsAccumulator accumulator{lap_provenance(), 10.0, 3};
    for (int tick = 0; tick <= 29; ++tick) {
        const auto time = 0.5 * tick;
        accumulator.observe(
            {time, std::fmod(2.0 * time, 10.0), 1.0, 0.1, false, false});
    }
    auto partial = accumulator.summary();
    EXPECT_FALSE(partial.lap_completed);
    EXPECT_EQ(partial.laps_completed, 2U);
    EXPECT_EQ(partial.lap_time, 0.0);

    accumulator.observe({15.0, 0.0, 1.0, 0.1, false, false});
    const auto result = accumulator.summary();
    EXPECT_TRUE(result.lap_completed);
    EXPECT_EQ(result.laps_completed, 3U);
    ASSERT_EQ(result.lap_times.size(), 3U);
    for (const auto lap : result.lap_times) {
        EXPECT_DOUBLE_EQ(lap, 5.0);
    }
    EXPECT_DOUBLE_EQ(result.lap_time, 5.0);  // the first lap's, as with laps=1
}

// METRICS-1070: a backward jump of s (the sim teleports a colliding car to its
// last checkpoint) gives the distance back, so the lap that holds it is
// longer by what was replayed.
TEST(MetricsAccumulatorTest, ABackwardTeleportSubtractsDistance) {
    MetricsAccumulator accumulator{lap_provenance(), 10.0, 2};
    // Steps stay under L/2 so none reads as a wrap. 0 -> 4 at 2 m/s, a
    // teleport back to 1 (3 m replayed), then forward to the line at t = 7.
    accumulator.observe({0.0, 0.0, 1.0, 0.0, false, false});
    accumulator.observe({2.0, 4.0, 1.0, 0.0, false, false});
    accumulator.observe({2.5, 1.0, 1.0, 0.0, false, false});
    accumulator.observe({4.5, 5.0, 1.0, 0.0, false, false});
    accumulator.observe({6.5, 9.0, 1.0, 0.0, false, false});
    accumulator.observe({7.0, 0.0, 1.0, 0.0, false, false});
    EXPECT_EQ(accumulator.summary().laps_completed, 1U);
    EXPECT_DOUBLE_EQ(accumulator.summary().lap_times.at(0), 7.0);
    accumulator.observe({9.5, 3.0, 1.0, 0.0, false, false});
    accumulator.observe({11.0, 7.0, 1.0, 0.0, false, false});
    accumulator.observe({12.5, 0.0, 1.0, 0.0, false, false});
    const auto result = accumulator.summary();
    ASSERT_EQ(result.lap_times.size(), 2U);
    EXPECT_DOUBLE_EQ(result.lap_times[1], 5.5);
    EXPECT_TRUE(result.lap_completed);
}

// METRICS-1080: laps < 1 is refused; laps = 1 (the default) reports the one
// lap as lap_times == {lap_time} and serialises exactly as before.
TEST(MetricsAccumulatorTest, OneLapIsTodaysBehaviourAndZeroLapsIsRefused) {
    EXPECT_THROW((MetricsAccumulator{lap_provenance(), 10.0, 0}),
                 std::invalid_argument);

    MetricsAccumulator by_default{lap_provenance(), 10.0};
    MetricsAccumulator explicit_one{lap_provenance(), 10.0, 1};
    for (auto *accumulator : {&by_default, &explicit_one}) {
        accumulator->observe({1.0, 8.0, 0.50, 0.10, false, false});
        accumulator->observe({2.0, 9.0, 0.40, 0.40, true, true});
        accumulator->observe({3.0, 0.5, 0.30, 0.20, true, true});
        accumulator->observe({4.0, 4.0, 0.20, 0.30, false, false});
        accumulator->observe({5.0, 8.1, 0.25, 0.00, true, true});
    }
    const auto result = by_default.summary();
    ASSERT_EQ(result.lap_times.size(), 1U);
    EXPECT_DOUBLE_EQ(result.lap_times[0], result.lap_time);
    EXPECT_DOUBLE_EQ(result.lap_time, 4.0);
    EXPECT_EQ(result.laps_completed, 1U);
    // The node's JSON file is the byte-for-byte contract of every golden.
    EXPECT_EQ(to_json(result, 5123456789, "map"),
              to_json(explicit_one.summary(), 5123456789, "map"));
    EXPECT_EQ(to_json(result, 5123456789, "map"),
              "{\"header\":{\"stamp\":{\"sec\":5,\"nanosec\":123456789},"
              "\"frame_id\":\"map\"},\"source\":\"source\","
              "\"scenario_id\":\"scenario\",\"seed\":1,\"timestep_ratio\":1,"
              "\"lap_completed\":true,\"lap_time\":4,\"collision_count\":2,"
              "\"minimum_wall_clearance\":0.20000000000000001,"
              "\"maximum_tracking_error\":0.40000000000000002,"
              "\"p95_tracking_error\":0.38,\"control_saturation_events\":2,"
              "\"code_revision\":\"revision\",\"container_image_digest\":"
              "\"digest\",\"track_version\":\"track\","
              "\"vehicle_parameter_version\":\"vehicle\"}");
}

}  // namespace
}  // namespace racing_metrics
