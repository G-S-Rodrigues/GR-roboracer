#pragma once

#include <filesystem>
#include <memory>

namespace racing_common {

struct CartesianPose {
    double x{};
    double y{};
    double psi{};
};

struct FrenetPoint {
    double s{};
    double d{};
    double heading_error{};
};

class Track {
   public:
    static Track from_yaml(const std::filesystem::path &path);

    FrenetPoint to_frenet(const CartesianPose &pose) const;
    CartesianPose to_cartesian(const FrenetPoint &point) const;
    double curvature_at(double s) const;
    bool is_inside(const FrenetPoint &point, double vehicle_half_width) const;

   private:
    struct Data;
    explicit Track(std::shared_ptr<const Data> data);

    std::shared_ptr<const Data> data_;
};

}  // namespace racing_common
