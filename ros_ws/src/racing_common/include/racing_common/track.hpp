#pragma once

#include <filesystem>
#include <memory>

namespace racing_common {

struct CartesianPose {
    double x{};
    double y{};
    double psi{};
};

// Moves a pose `distance` metres along its own heading; yaw is unchanged.
// The gym's state is the centre of gravity (cog_link); base_link is the rear
// axle, lr behind it. So base_link = translate_along_heading(cog, -lr) and
// cog_link = translate_along_heading(base_link, +lr) (part 3.1).
CartesianPose translate_along_heading(const CartesianPose &pose,
                                      double distance);

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
    /// Centerline arc length: the distance one lap covers.
    double length() const;
    bool is_inside(const FrenetPoint &point, double vehicle_half_width) const;

   private:
    struct Data;
    explicit Track(std::shared_ptr<const Data> data);

    std::shared_ptr<const Data> data_;
};

}  // namespace racing_common
