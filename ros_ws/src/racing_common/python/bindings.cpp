#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "racing_common/action_map.hpp"
#include "racing_common/track.hpp"

namespace py = pybind11;

PYBIND11_MODULE(racing_common, module) {
    module.doc() = "Python bindings for the shared C++ racing core";

    py::enum_<racing_common::LongitudinalAction>(module, "LongitudinalAction")
        .value("ACCELERATION", racing_common::LongitudinalAction::ACCELERATION)
        .value("VELOCITY", racing_common::LongitudinalAction::VELOCITY);

    py::enum_<racing_common::SteeringAction>(module, "SteeringAction")
        .value("STEERING_ANGLE", racing_common::SteeringAction::STEERING_ANGLE)
        .value("STEERING_VELOCITY",
               racing_common::SteeringAction::STEERING_VELOCITY);

    py::class_<racing_common::DriveCommand>(module, "DriveCommand")
        .def(py::init<>())
        .def_readwrite("steering_angle",
                       &racing_common::DriveCommand::steering_angle)
        .def_readwrite("steering_angle_velocity",
                       &racing_common::DriveCommand::steering_angle_velocity)
        .def_readwrite("speed", &racing_common::DriveCommand::speed)
        .def_readwrite("acceleration",
                       &racing_common::DriveCommand::acceleration);

    py::class_<racing_common::EnvActionSpec>(module, "EnvActionSpec")
        .def(py::init<racing_common::LongitudinalAction,
                      racing_common::SteeringAction>())
        .def_readwrite("longitudinal",
                       &racing_common::EnvActionSpec::longitudinal)
        .def_readwrite("steering", &racing_common::EnvActionSpec::steering);

    module.def("apply_drive_command", &racing_common::apply_drive_command,
               py::arg("command"), py::arg("spec"));

    py::class_<racing_common::CartesianPose>(module, "CartesianPose")
        .def(py::init<double, double, double>(), py::arg("x") = 0.0,
             py::arg("y") = 0.0, py::arg("psi") = 0.0)
        .def_readwrite("x", &racing_common::CartesianPose::x)
        .def_readwrite("y", &racing_common::CartesianPose::y)
        .def_readwrite("psi", &racing_common::CartesianPose::psi);

    py::class_<racing_common::FrenetPoint>(module, "FrenetPoint")
        .def(py::init<double, double, double>(), py::arg("s") = 0.0,
             py::arg("d") = 0.0, py::arg("heading_error") = 0.0)
        .def_readwrite("s", &racing_common::FrenetPoint::s)
        .def_readwrite("d", &racing_common::FrenetPoint::d)
        .def_readwrite("heading_error",
                       &racing_common::FrenetPoint::heading_error);

    py::class_<racing_common::Track>(module, "Track")
        .def_static("from_yaml", &racing_common::Track::from_yaml)
        .def("to_frenet", &racing_common::Track::to_frenet)
        .def("to_cartesian", &racing_common::Track::to_cartesian)
        .def("curvature_at", &racing_common::Track::curvature_at)
        .def("is_inside", &racing_common::Track::is_inside);
}
