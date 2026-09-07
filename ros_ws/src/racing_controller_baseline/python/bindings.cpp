#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "racing_controller_baseline/pure_pursuit.hpp"

namespace py = pybind11;
using namespace racing_controller_baseline;

PYBIND11_MODULE(racing_controller_baseline, module) {
    module.doc() =
        "Python binding for the authoritative C++ baseline controller";

    py::class_<VehicleState>(module, "VehicleState")
        .def(py::init<double, double, double>(), py::arg("x") = 0.0,
             py::arg("y") = 0.0, py::arg("yaw") = 0.0)
        .def_readwrite("x", &VehicleState::x)
        .def_readwrite("y", &VehicleState::y)
        .def_readwrite("yaw", &VehicleState::yaw);

    py::class_<TrajectoryPoint>(module, "TrajectoryPoint")
        .def(py::init<double, double>(), py::arg("x") = 0.0, py::arg("y") = 0.0)
        .def_readwrite("x", &TrajectoryPoint::x)
        .def_readwrite("y", &TrajectoryPoint::y);

    py::class_<ControllerParams>(module, "ControllerParams")
        .def(py::init<>())
        .def_readwrite("lookahead_distance",
                       &ControllerParams::lookahead_distance)
        .def_readwrite("wheelbase", &ControllerParams::wheelbase)
        .def_readwrite("minimum_speed", &ControllerParams::minimum_speed)
        .def_readwrite("maximum_speed", &ControllerParams::maximum_speed)
        .def_readwrite("curvature_speed_gain",
                       &ControllerParams::curvature_speed_gain)
        .def_readwrite("closed_trajectory",
                       &ControllerParams::closed_trajectory);

    py::class_<DriveCommand>(module, "DriveCommand")
        .def_readonly("steering_angle", &DriveCommand::steering_angle)
        .def_readonly("speed", &DriveCommand::speed)
        .def_readonly("curvature", &DriveCommand::curvature)
        .def_readonly("target_x", &DriveCommand::target_x)
        .def_readonly("target_y", &DriveCommand::target_y);

    py::class_<PurePursuit>(module, "PurePursuit")
        .def_static("compute", &PurePursuit::compute, py::arg("state"),
                    py::arg("trajectory"), py::arg("params"));

    module.def("curvature_speed_schedule", &curvature_speed_schedule,
               py::arg("kappa"), py::arg("params"));
}
