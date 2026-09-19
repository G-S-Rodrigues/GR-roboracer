#include "racing_metrics/metrics_accumulator.hpp"

#include <gtest/gtest.h>

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

}  // namespace
}  // namespace racing_metrics
