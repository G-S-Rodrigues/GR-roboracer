#include "racing_common/track.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace racing_common {
namespace {

constexpr double k_pi = 3.14159265358979323846;
constexpr double k_two_pi = 2.0 * k_pi;

double wrap_angle(double angle) { return std::remainder(angle, k_two_pi); }

double clamp_unit(double value) { return std::clamp(value, 0.0, 1.0); }

double interpolate(double start, double end, double ratio) {
    return start + (end - start) * ratio;
}

}  // namespace

struct Track::Data {
    struct Point {
        double x;
        double y;
        double curvature;
        double left_width;
        double right_width;
    };

    struct Segment {
        std::size_t start_index;
        double start_s;
        double length;
        double unit_x;
        double unit_y;
        double heading;
    };

    std::vector<Point> points;
    std::vector<Segment> segments;
    double length{};
    bool closed{};
};

Track::Track(std::shared_ptr<const Data> data) : data_(std::move(data)) {}

Track Track::from_yaml(const std::filesystem::path &path) {
    try {
        const auto root = YAML::LoadFile(path.string());
        if (!root["format_version"] || root["format_version"].as<int>() != 1) {
            throw std::runtime_error("unsupported track format_version");
        }
        if (!root["metadata"] || !root["metadata"]["name"] ||
            !root["metadata"]["closed"] || !root["metadata"]["units"] ||
            root["metadata"]["units"].as<std::string>() != "meters") {
            throw std::runtime_error("invalid track metadata");
        }

        const auto centerline = root["centerline"];
        if (!centerline || !centerline.IsSequence() || centerline.size() < 2) {
            throw std::runtime_error("centerline requires at least two points");
        }

        auto data = std::make_shared<Data>();
        data->closed = root["metadata"]["closed"].as<bool>();
        data->points.reserve(centerline.size());
        for (const auto &entry : centerline) {
            if (!entry.IsSequence() || entry.size() != 5) {
                throw std::runtime_error(
                    "each centerline point must have exactly five values");
            }
            Data::Point point{entry[0].as<double>(), entry[1].as<double>(),
                              entry[2].as<double>(), entry[3].as<double>(),
                              entry[4].as<double>()};
            if (point.left_width <= 0.0 || point.right_width <= 0.0) {
                throw std::runtime_error("boundary widths must be positive");
            }
            data->points.push_back(point);
        }

        const auto segment_count =
            data->closed ? data->points.size() : data->points.size() - 1;
        data->segments.reserve(segment_count);
        for (std::size_t index = 0; index < segment_count; ++index) {
            const auto next_index = (index + 1) % data->points.size();
            const auto dx = data->points[next_index].x - data->points[index].x;
            const auto dy = data->points[next_index].y - data->points[index].y;
            const auto length = std::hypot(dx, dy);
            if (length <= std::numeric_limits<double>::epsilon()) {
                throw std::runtime_error(
                    "centerline segments must have length");
            }
            data->segments.push_back({index, data->length, length, dx / length,
                                      dy / length, std::atan2(dy, dx)});
            data->length += length;
        }
        return Track{std::move(data)};
    } catch (const YAML::Exception &error) {
        throw std::runtime_error("failed to load track '" + path.string() +
                                 "': " + error.what());
    }
}

FrenetPoint Track::to_frenet(const CartesianPose &pose) const {
    const Data::Segment *closest = nullptr;
    double closest_ratio = 0.0;
    double minimum_distance_squared = std::numeric_limits<double>::infinity();

    for (const auto &segment : data_->segments) {
        const auto &start = data_->points[segment.start_index];
        const auto offset_x = pose.x - start.x;
        const auto offset_y = pose.y - start.y;
        const auto ratio =
            clamp_unit((offset_x * segment.unit_x + offset_y * segment.unit_y) /
                       segment.length);
        const auto projected_x =
            start.x + ratio * segment.length * segment.unit_x;
        const auto projected_y =
            start.y + ratio * segment.length * segment.unit_y;
        const auto distance_squared = std::pow(pose.x - projected_x, 2) +
                                      std::pow(pose.y - projected_y, 2);
        if (distance_squared < minimum_distance_squared) {
            minimum_distance_squared = distance_squared;
            closest = &segment;
            closest_ratio = ratio;
        }
    }

    const auto &start = data_->points[closest->start_index];
    const auto projected_x =
        start.x + closest_ratio * closest->length * closest->unit_x;
    const auto projected_y =
        start.y + closest_ratio * closest->length * closest->unit_y;
    const auto normal_x = -closest->unit_y;
    const auto normal_y = closest->unit_x;
    return {
        closest->start_s + closest_ratio * closest->length,
        (pose.x - projected_x) * normal_x + (pose.y - projected_y) * normal_y,
        wrap_angle(pose.psi - closest->heading)};
}

CartesianPose Track::to_cartesian(const FrenetPoint &point) const {
    auto s = point.s;
    if (data_->closed) {
        s = std::fmod(s, data_->length);
        if (s < 0.0) {
            s += data_->length;
        }
    } else {
        s = std::clamp(s, 0.0, data_->length);
    }

    const auto segment_iterator =
        std::upper_bound(data_->segments.begin(), data_->segments.end(), s,
                         [](double value, const Data::Segment &segment) {
                             return value < segment.start_s;
                         });
    const auto &segment = segment_iterator == data_->segments.begin()
                              ? data_->segments.front()
                              : *std::prev(segment_iterator);
    const auto distance = std::min(s - segment.start_s, segment.length);
    const auto &start = data_->points[segment.start_index];
    const auto normal_x = -segment.unit_y;
    const auto normal_y = segment.unit_x;
    return {start.x + distance * segment.unit_x + point.d * normal_x,
            start.y + distance * segment.unit_y + point.d * normal_y,
            wrap_angle(segment.heading + point.heading_error)};
}

double Track::curvature_at(double s) const {
    const auto pose = to_cartesian({s, 0.0, 0.0});
    const auto frenet = to_frenet(pose);
    const auto segment_iterator = std::upper_bound(
        data_->segments.begin(), data_->segments.end(), frenet.s,
        [](double value, const Data::Segment &segment) {
            return value < segment.start_s;
        });
    const auto &segment = segment_iterator == data_->segments.begin()
                              ? data_->segments.front()
                              : *std::prev(segment_iterator);
    const auto next_index = (segment.start_index + 1) % data_->points.size();
    const auto ratio =
        clamp_unit((frenet.s - segment.start_s) / segment.length);
    return interpolate(data_->points[segment.start_index].curvature,
                       data_->points[next_index].curvature, ratio);
}

double Track::length() const { return data_->length; }

bool Track::is_inside(const FrenetPoint &point,
                      double vehicle_half_width) const {
    if (vehicle_half_width < 0.0) {
        return false;
    }
    const auto center = to_cartesian({point.s, 0.0, 0.0});
    const auto normalized = to_frenet(center);
    const auto segment_iterator = std::upper_bound(
        data_->segments.begin(), data_->segments.end(), normalized.s,
        [](double value, const Data::Segment &segment) {
            return value < segment.start_s;
        });
    const auto &segment = segment_iterator == data_->segments.begin()
                              ? data_->segments.front()
                              : *std::prev(segment_iterator);
    const auto next_index = (segment.start_index + 1) % data_->points.size();
    const auto ratio =
        clamp_unit((normalized.s - segment.start_s) / segment.length);
    const auto left_width =
        interpolate(data_->points[segment.start_index].left_width,
                    data_->points[next_index].left_width, ratio);
    const auto right_width =
        interpolate(data_->points[segment.start_index].right_width,
                    data_->points[next_index].right_width, ratio);
    return point.d + vehicle_half_width <= left_width &&
           point.d - vehicle_half_width >= -right_width;
}

}  // namespace racing_common
