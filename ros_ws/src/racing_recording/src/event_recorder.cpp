#include "racing_recording/event_recorder.hpp"

#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace racing_recording {
namespace {

constexpr std::int64_t nanoseconds_per_second = 1000000000;

std::string escape_json(std::string_view value) {
    std::ostringstream output;
    for (const auto character : value) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << character;
                break;
        }
    }
    return output.str();
}

}  // namespace

EventRecorder::EventRecorder(std::filesystem::path output_path,
                             const RecordingManifest &manifest)
    : output_path_(std::move(output_path)) {
    if (output_path_.empty()) {
        throw std::invalid_argument("output_path must not be empty");
    }
    if (manifest.timestep_ratio == 0U) {
        throw std::invalid_argument("timestep_ratio must be positive");
    }
    if (output_path_.has_parent_path()) {
        std::filesystem::create_directories(output_path_.parent_path());
    }
    output_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("cannot open recording: " +
                                 output_path_.string());
    }
    write_manifest(manifest);
    flush();
}

EventRecorder::~EventRecorder() { output_.flush(); }

void EventRecorder::write_manifest(const RecordingManifest &manifest) {
    output_ << "{\"record_type\":\"manifest\",\"schema_version\":1,"
            << "\"source\":\"" << escape_json(manifest.source) << "\","
            << "\"scenario_id\":\"" << escape_json(manifest.scenario_id)
            << "\",\"seed\":" << manifest.seed
            << ",\"timestep_ratio\":" << manifest.timestep_ratio
            << ",\"code_revision\":\"" << escape_json(manifest.code_revision)
            << "\",\"container_image_digest\":\""
            << escape_json(manifest.container_image_digest)
            << "\",\"track_version\":\"" << escape_json(manifest.track_version)
            << "\",\"vehicle_parameter_version\":\""
            << escape_json(manifest.vehicle_parameter_version)
            << "\",\"controller_configuration\":\""
            << escape_json(manifest.controller_configuration)
            << "\",\"platform\":\"" << escape_json(manifest.platform)
            << "\"}\n";
    if (!output_) {
        throw std::runtime_error("cannot write recording manifest");
    }
}

void EventRecorder::record(const std::string &topic,
                           const std::string &message_type,
                           std::int64_t timestamp_nanoseconds,
                           const std::string &json_payload) {
    if (topic.empty() || message_type.empty()) {
        throw std::invalid_argument("topic and message_type must not be empty");
    }
    if (timestamp_nanoseconds < 0) {
        throw std::invalid_argument("timestamp must be nonnegative");
    }
    if (json_payload.empty() ||
        (json_payload.front() != '{' && json_payload.front() != '[')) {
        throw std::invalid_argument("payload must be a JSON object or array");
    }
    const auto seconds = timestamp_nanoseconds / nanoseconds_per_second;
    const auto nanoseconds = timestamp_nanoseconds % nanoseconds_per_second;
    output_ << "{\"record_type\":\"event\",\"stamp\":{\"sec\":" << seconds
            << ",\"nanosec\":" << nanoseconds << "},\"topic\":\""
            << escape_json(topic) << "\",\"message_type\":\""
            << escape_json(message_type) << "\",\"payload\":" << json_payload
            << "}\n";
    if (!output_) {
        throw std::runtime_error("cannot write recording event");
    }
}

void EventRecorder::flush() {
    output_.flush();
    if (!output_) {
        throw std::runtime_error("cannot flush recording");
    }
}

const std::filesystem::path &EventRecorder::output_path() const {
    return output_path_;
}

}  // namespace racing_recording
