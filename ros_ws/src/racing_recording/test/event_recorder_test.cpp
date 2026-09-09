#include "racing_recording/event_recorder.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace racing_recording {
namespace {

TEST(EventRecorderTest, WritesManifestBeforeReplayableEvents) {
    const auto output_path = std::filesystem::temp_directory_path() /
                             "racing_recording_event_recorder_test.jsonl";
    std::filesystem::remove(output_path);
    const RecordingManifest manifest{
        "recording_test",  "synthetic",    42,         4,
        "revision-abc",    "sha256:image", "track-v1", "vehicle-v2",
        "pure-pursuit-v1", "linux-x86_64"};

    {
        EventRecorder recorder{output_path, manifest};
        recorder.record("/drive", "ackermann_msgs/AckermannDriveStamped",
                        1500000000, "{\"speed\":2.5}");
        recorder.record("/safety/status", "racing_interfaces/SafetyStatus",
                        1750000000, "{\"active_clamps\":0}");
        recorder.flush();

        std::ifstream input{output_path};
        const std::string contents{std::istreambuf_iterator<char>{input},
                                   std::istreambuf_iterator<char>{}};
        const auto manifest_position =
            contents.find("\"record_type\":\"manifest\"");
        const auto drive_position = contents.find("\"topic\":\"/drive\"");
        const auto safety_position =
            contents.find("\"topic\":\"/safety/status\"");
        ASSERT_NE(manifest_position, std::string::npos);
        ASSERT_NE(drive_position, std::string::npos);
        ASSERT_NE(safety_position, std::string::npos);
        EXPECT_LT(manifest_position, drive_position);
        EXPECT_LT(drive_position, safety_position);
        EXPECT_NE(contents.find("\"scenario_id\":\"synthetic\""),
                  std::string::npos);
        EXPECT_NE(contents.find("\"seed\":42"), std::string::npos);
        EXPECT_NE(contents.find("\"timestep_ratio\":4"), std::string::npos);
        EXPECT_NE(contents.find("\"container_image_digest\":"
                                "\"sha256:image\""),
                  std::string::npos);
        EXPECT_NE(contents.find("\"stamp\":{\"sec\":1,"
                                "\"nanosec\":500000000}"),
                  std::string::npos);
        EXPECT_NE(contents.find("\"payload\":{\"speed\":2.5}"),
                  std::string::npos);
    }
    std::filesystem::remove(output_path);
}

}  // namespace
}  // namespace racing_recording
