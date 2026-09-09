#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace racing_recording {

struct RecordingManifest {
    std::string source;
    std::string scenario_id;
    std::uint64_t seed{0};
    std::uint32_t timestep_ratio{0};
    std::string code_revision;
    std::string container_image_digest;
    std::string track_version;
    std::string vehicle_parameter_version;
    std::string controller_configuration;
    std::string platform;
};

class EventRecorder {
   public:
    EventRecorder(std::filesystem::path output_path,
                  const RecordingManifest &manifest);
    ~EventRecorder();

    EventRecorder(const EventRecorder &) = delete;
    EventRecorder &operator=(const EventRecorder &) = delete;
    EventRecorder(EventRecorder &&) = delete;
    EventRecorder &operator=(EventRecorder &&) = delete;

    void record(const std::string &topic, const std::string &message_type,
                std::int64_t timestamp_nanoseconds,
                const std::string &json_payload);
    void flush();

    [[nodiscard]] const std::filesystem::path &output_path() const;

   private:
    void write_manifest(const RecordingManifest &manifest);

    std::filesystem::path output_path_;
    std::ofstream output_;
};

}  // namespace racing_recording
