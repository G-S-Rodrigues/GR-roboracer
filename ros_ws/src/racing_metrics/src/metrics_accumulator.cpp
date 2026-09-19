#include "racing_metrics/metrics_accumulator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace racing_metrics {
namespace {

void require_finite_nonnegative(double value, const char *name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string{name} +
                                    " must be finite and nonnegative");
    }
}

double percentile_95(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const auto position = 0.95 * static_cast<double>(values.size() - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = static_cast<std::size_t>(std::ceil(position));
    const auto fraction = position - static_cast<double>(lower);
    return values[lower] + (values[upper] - values[lower]) * fraction;
}

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

MetricsAccumulator::MetricsAccumulator(RunProvenance provenance,
                                       double track_length)
    : provenance_(std::move(provenance)), track_length_(track_length) {
    if (!std::isfinite(track_length_) || track_length_ <= 0.0) {
        throw std::invalid_argument("track_length must be finite and positive");
    }
}

void MetricsAccumulator::observe(const MetricsSample &sample) {
    if (lap_completed_) {
        return;
    }
    require_finite_nonnegative(sample.elapsed_time, "elapsed_time");
    require_finite_nonnegative(sample.wall_clearance, "wall_clearance");
    require_finite_nonnegative(sample.tracking_error, "tracking_error");
    if (!std::isfinite(sample.s)) {
        throw std::invalid_argument("s must be finite");
    }
    // A republished simulated instant (a sim held at t=0, or in STEPPED
    // mode) is one sample, not many: otherwise how long the hold lasted in
    // wall time would weight the tracking-error percentile.
    if (started_ && sample.elapsed_time == last_time_) {
        return;
    }
    last_time_ = sample.elapsed_time;

    auto normalized_s = std::fmod(sample.s, track_length_);
    if (normalized_s < 0.0) {
        normalized_s += track_length_;
    }

    if (!started_) {
        started_ = true;
        start_time_ = sample.elapsed_time;
        last_s_ = normalized_s;
        minimum_wall_clearance_ = sample.wall_clearance;
    } else {
        if (sample.elapsed_time < start_time_ + lap_time_) {
            throw std::invalid_argument("elapsed_time must be monotonic");
        }
        auto delta = normalized_s - last_s_;
        if (delta < -track_length_ / 2.0) {
            delta += track_length_;
        } else if (delta > track_length_ / 2.0) {
            delta -= track_length_;
        }
        distance_travelled_ += delta;
        last_s_ = normalized_s;
        minimum_wall_clearance_ =
            std::min(minimum_wall_clearance_, sample.wall_clearance);
    }

    lap_time_ = sample.elapsed_time - start_time_;
    if (!lap_completed_ && distance_travelled_ >= track_length_) {
        lap_completed_ = true;
    }
    if (sample.colliding && !was_colliding_) {
        ++collision_count_;
    }
    if (sample.control_saturated && !was_saturated_) {
        ++control_saturation_events_;
    }
    was_colliding_ = sample.colliding;
    was_saturated_ = sample.control_saturated;
    tracking_errors_.push_back(sample.tracking_error);
}

ScenarioMetricsData MetricsAccumulator::summary() const {
    ScenarioMetricsData result;
    result.source = provenance_.source;
    result.scenario_id = provenance_.scenario_id;
    result.seed = provenance_.seed;
    result.timestep_ratio = provenance_.timestep_ratio;
    result.lap_completed = lap_completed_;
    result.lap_time = lap_completed_ ? lap_time_ : 0.0;
    result.collision_count = collision_count_;
    result.minimum_wall_clearance = started_ ? minimum_wall_clearance_ : 0.0;
    result.maximum_tracking_error =
        tracking_errors_.empty() ? 0.0
                                 : *std::max_element(tracking_errors_.begin(),
                                                     tracking_errors_.end());
    result.p95_tracking_error = percentile_95(tracking_errors_);
    result.control_saturation_events = control_saturation_events_;
    result.code_revision = provenance_.code_revision;
    result.container_image_digest = provenance_.container_image_digest;
    result.track_version = provenance_.track_version;
    result.vehicle_parameter_version = provenance_.vehicle_parameter_version;
    return result;
}

std::string to_json(const ScenarioMetricsData &metrics,
                    std::int64_t timestamp_nanoseconds,
                    const std::string &frame_id) {
    constexpr std::int64_t nanoseconds_per_second = 1000000000;
    const auto seconds = timestamp_nanoseconds / nanoseconds_per_second;
    const auto nanoseconds = timestamp_nanoseconds % nanoseconds_per_second;
    std::ostringstream output;
    output << std::setprecision(17) << '{'
           << "\"header\":{\"stamp\":{\"sec\":" << seconds
           << ",\"nanosec\":" << nanoseconds << "},\"frame_id\":\""
           << escape_json(frame_id) << "\"}," << "\"source\":\""
           << escape_json(metrics.source) << "\"," << "\"scenario_id\":\""
           << escape_json(metrics.scenario_id) << "\",\"seed\":" << metrics.seed
           << ",\"timestep_ratio\":" << metrics.timestep_ratio
           << ",\"lap_completed\":"
           << (metrics.lap_completed ? "true" : "false")
           << ",\"lap_time\":" << metrics.lap_time
           << ",\"collision_count\":" << metrics.collision_count
           << ",\"minimum_wall_clearance\":" << metrics.minimum_wall_clearance
           << ",\"maximum_tracking_error\":" << metrics.maximum_tracking_error
           << ",\"p95_tracking_error\":" << metrics.p95_tracking_error
           << ",\"control_saturation_events\":"
           << metrics.control_saturation_events << ",\"code_revision\":\""
           << escape_json(metrics.code_revision)
           << "\",\"container_image_digest\":\""
           << escape_json(metrics.container_image_digest)
           << "\",\"track_version\":\"" << escape_json(metrics.track_version)
           << "\",\"vehicle_parameter_version\":\""
           << escape_json(metrics.vehicle_parameter_version) << "\"}";
    return output.str();
}

}  // namespace racing_metrics
