#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace racing_metrics {

struct RunProvenance {
    std::string source;
    std::string scenario_id;
    std::uint64_t seed{0};
    std::uint32_t timestep_ratio{0};
    std::string code_revision;
    std::string container_image_digest;
    std::string track_version;
    std::string vehicle_parameter_version;
};

struct MetricsSample {
    double elapsed_time{0.0};
    double s{0.0};
    double wall_clearance{0.0};
    double tracking_error{0.0};
    bool colliding{false};
    bool control_saturated{false};
};

struct ScenarioMetricsData {
    std::string source;
    std::string scenario_id;
    std::uint64_t seed{0};
    std::uint32_t timestep_ratio{0};
    bool lap_completed{false};
    double lap_time{0.0};
    std::uint32_t collision_count{0};
    double minimum_wall_clearance{0.0};
    double maximum_tracking_error{0.0};
    double p95_tracking_error{0.0};
    std::uint32_t control_saturation_events{0};
    std::string code_revision;
    std::string container_image_digest;
    std::string track_version;
    std::string vehicle_parameter_version;
    // Appended by #44 part 10; to_json does not serialise them.
    std::uint32_t laps_completed{0};
    std::vector<double> lap_times;  // one duration per completed lap
};

std::string to_json(const ScenarioMetricsData &metrics,
                    std::int64_t timestamp_nanoseconds,
                    const std::string &frame_id);

class MetricsAccumulator {
   public:
    // `laps` (>= 1) is how many laps complete the run. lap_time stays the
    // first lap's time, so laps = 1 is exactly the single-lap behaviour.
    MetricsAccumulator(RunProvenance provenance, double track_length,
                       std::uint32_t laps = 1);

    void observe(const MetricsSample &sample);
    [[nodiscard]] ScenarioMetricsData summary() const;

   private:
    RunProvenance provenance_;
    double track_length_;
    std::uint32_t laps_;
    double start_time_{0.0};
    double last_time_{0.0};
    double last_s_{0.0};
    double distance_travelled_{0.0};
    double lap_time_{0.0};
    double minimum_wall_clearance_{0.0};
    std::uint32_t collision_count_{0};
    std::uint32_t control_saturation_events_{0};
    bool started_{false};
    bool lap_completed_{false};
    bool was_colliding_{false};
    bool was_saturated_{false};
    std::vector<double> tracking_errors_;
    std::vector<double> lap_times_;  // durations of the completed laps
    double last_lap_end_{0.0};       // elapsed_time at the previous crossing
};

}  // namespace racing_metrics
