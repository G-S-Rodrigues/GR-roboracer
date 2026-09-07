#include "racing_common/track.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>

namespace {

std::filesystem::path track_path() {
    return std::filesystem::path{RACING_COMMON_TEST_TRACK};
}

std::filesystem::path unsupported_track_path() {
    return std::filesystem::path{RACING_COMMON_UNSUPPORTED_TEST_TRACK};
}

TEST(TrackTest, Common1010FrenetCartesianRoundTrip) {
    const auto track = racing_common::Track::from_yaml(track_path());
    const racing_common::FrenetPoint original{5.0, 0.75, -0.2};

    const auto pose = track.to_cartesian(original);
    const auto actual = track.to_frenet(pose);

    EXPECT_NEAR(actual.s, original.s, 1e-9);
    EXPECT_NEAR(actual.d, original.d, 1e-9);
    EXPECT_NEAR(actual.heading_error, original.heading_error, 1e-9);
}

TEST(TrackTest, Common1020CurvatureMatchesAnalyticCircle) {
    const auto track = racing_common::Track::from_yaml(track_path());

    EXPECT_NEAR(track.curvature_at(0.0), 0.1, 1e-12);
    EXPECT_NEAR(track.curvature_at(17.0), 0.1, 1e-12);
    EXPECT_NEAR(track.curvature_at(1000.0), 0.1, 1e-12);
}

TEST(TrackTest, Common1030ExactBoundaryIncludesVehicleEnvelope) {
    const auto track = racing_common::Track::from_yaml(track_path());

    EXPECT_TRUE(track.is_inside({0.0, 1.5, 0.0}, 0.5));
    EXPECT_FALSE(track.is_inside({0.0, 1.500001, 0.0}, 0.5));
    EXPECT_TRUE(track.is_inside({0.0, -2.5, 0.0}, 0.5));
    EXPECT_FALSE(track.is_inside({0.0, -2.500001, 0.0}, 0.5));
}

TEST(TrackTest, Common1040LoadsAndValidatesCanonicalYaml) {
    EXPECT_NO_THROW(racing_common::Track::from_yaml(track_path()));
    EXPECT_THROW(racing_common::Track::from_yaml(unsupported_track_path()),
                 std::runtime_error);
    EXPECT_THROW(racing_common::Track::from_yaml(track_path().parent_path() /
                                                 "missing-track.yaml"),
                 std::runtime_error);
}

}  // namespace
