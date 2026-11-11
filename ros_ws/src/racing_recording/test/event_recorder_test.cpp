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

// REC-1010: a SIGKILLed recorder never runs its destructor, so a record must
// be on disk, whole, as soon as record() returns (gotcha: buffered tail).
TEST(EventRecorderTest, Rec1010EveryRecordIsOnDiskBeforeDestruction) {
    const auto output_path = std::filesystem::temp_directory_path() /
                             "racing_recording_rec1010_test.jsonl";
    std::filesystem::remove(output_path);
    const RecordingManifest manifest{
        "recording_test",  "synthetic",    42,         4,
        "revision-abc",    "sha256:image", "track-v1", "vehicle-v2",
        "pure-pursuit-v1", "linux-x86_64"};
    constexpr int event_count = 50;
    const std::string padding(150, 'x');

    EventRecorder recorder{output_path, manifest};
    for (int index = 0; index < event_count; ++index) {
        recorder.record("/ground_truth/odom", "nav_msgs/msg/Odometry",
                        1000000000LL + index,
                        "{\"index\":" + std::to_string(index) +
                            ",\"padding\":\"" + padding + "\"}");
    }

    // The recorder is still alive: nothing may be waiting in its buffer.
    std::ifstream input{output_path};
    std::string contents{std::istreambuf_iterator<char>{input},
                         std::istreambuf_iterator<char>{}};
    ASSERT_FALSE(contents.empty());
    EXPECT_EQ('\n', contents.back());
    int lines = 0;
    std::size_t start = 0;
    while (start < contents.size()) {
        auto end = contents.find('\n', start);
        if (end == std::string::npos) {
            end = contents.size();  // an unterminated last line
        }
        EXPECT_GT(end, start);
        EXPECT_EQ('}', contents[end - 1]) << "line " << lines << " is cut";
        ++lines;
        start = end + 1;
    }
    EXPECT_EQ(event_count + 1, lines);
    std::filesystem::remove(output_path);
}

}  // namespace
}  // namespace racing_recording
