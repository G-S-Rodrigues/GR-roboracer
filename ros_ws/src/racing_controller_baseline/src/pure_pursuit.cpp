#include "racing_controller_baseline/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace racing_controller_baseline {
namespace {

struct Segment {
    TrajectoryPoint start;
    TrajectoryPoint end;
    double start_distance;
    double length;
};

void validate(const Trajectory &trajectory, const ControllerParams &params) {
    if (trajectory.empty()) {
        throw std::invalid_argument("trajectory must not be empty");
    }
    if (params.lookahead_distance <= 0.0) {
        throw std::invalid_argument("lookahead distance must be positive");
    }
    if (params.wheelbase <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
    if (params.minimum_speed < 0.0 ||
        params.maximum_speed < params.minimum_speed) {
        throw std::invalid_argument("speed limits are inconsistent");
    }
    if (params.curvature_speed_gain < 0.0) {
        throw std::invalid_argument(
            "curvature speed gain must be non-negative");
    }
}

std::vector<Segment> make_segments(const Trajectory &trajectory, bool closed) {
    std::vector<Segment> segments;
    if (trajectory.size() == 1) {
        return segments;
    }
    const auto count = closed ? trajectory.size() : trajectory.size() - 1;
    double distance = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const auto &start = trajectory[index];
        const auto &end = trajectory[(index + 1) % trajectory.size()];
        const auto length = std::hypot(end.x - start.x, end.y - start.y);
        if (length <= std::numeric_limits<double>::epsilon()) {
            continue;
        }
        segments.push_back({start, end, distance, length});
        distance += length;
    }
    return segments;
}

double closest_progress(const VehicleState &state,
                        const std::vector<Segment> &segments) {
    double progress = 0.0;
    double minimum_squared_distance = std::numeric_limits<double>::infinity();
    for (const auto &segment : segments) {
        const auto dx = segment.end.x - segment.start.x;
        const auto dy = segment.end.y - segment.start.y;
        const auto offset_x = state.x - segment.start.x;
        const auto offset_y = state.y - segment.start.y;
        const auto ratio = std::clamp(
            (offset_x * dx + offset_y * dy) / (segment.length * segment.length),
            0.0, 1.0);
        const auto projected_x = segment.start.x + ratio * dx;
        const auto projected_y = segment.start.y + ratio * dy;
        const auto squared_distance = std::pow(state.x - projected_x, 2) +
                                      std::pow(state.y - projected_y, 2);
        if (squared_distance < minimum_squared_distance) {
            minimum_squared_distance = squared_distance;
            progress = segment.start_distance + ratio * segment.length;
        }
    }
    return progress;
}

TrajectoryPoint point_at(double distance,
                         const std::vector<Segment> &segments) {
    for (const auto &segment : segments) {
        if (distance <= segment.start_distance + segment.length) {
            const auto ratio = std::clamp(
                (distance - segment.start_distance) / segment.length, 0.0, 1.0);
            return {
                segment.start.x + ratio * (segment.end.x - segment.start.x),
                segment.start.y + ratio * (segment.end.y - segment.start.y)};
        }
    }
    return segments.back().end;
}

}  // namespace

double curvature_speed_schedule(double kappa, const ControllerParams &params) {
    if (params.minimum_speed < 0.0 ||
        params.maximum_speed < params.minimum_speed ||
        params.curvature_speed_gain < 0.0) {
        throw std::invalid_argument("invalid speed schedule parameters");
    }
    const auto scheduled =
        params.maximum_speed /
        (1.0 + params.curvature_speed_gain * std::abs(kappa));
    return std::clamp(scheduled, params.minimum_speed, params.maximum_speed);
}

DriveCommand PurePursuit::compute(const VehicleState &state,
                                  const Trajectory &trajectory,
                                  const ControllerParams &params) {
    validate(trajectory, params);
    const auto segments = make_segments(trajectory, params.closed_trajectory);
    auto target = trajectory.front();
    if (!segments.empty()) {
        const auto total_distance =
            segments.back().start_distance + segments.back().length;
        auto target_distance =
            closest_progress(state, segments) + params.lookahead_distance;
        if (params.closed_trajectory) {
            target_distance = std::fmod(target_distance, total_distance);
        } else {
            target_distance = std::clamp(target_distance, 0.0, total_distance);
        }
        target = point_at(target_distance, segments);
    }

    const auto dx = target.x - state.x;
    const auto dy = target.y - state.y;
    const auto local_y = -std::sin(state.yaw) * dx + std::cos(state.yaw) * dy;
    const auto squared_distance = dx * dx + dy * dy;
    const auto curvature =
        squared_distance <= std::numeric_limits<double>::epsilon()
            ? 0.0
            : 2.0 * local_y / squared_distance;
    return {std::atan(params.wheelbase * curvature),
            curvature_speed_schedule(curvature, params), curvature, target.x,
            target.y};
}

}  // namespace racing_controller_baseline
