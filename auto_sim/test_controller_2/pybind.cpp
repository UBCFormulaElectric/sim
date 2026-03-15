#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controller.hpp"
#include "sim.hpp"
#include "types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(Controller, m, py::mod_gil_not_used())
{
    py::class_<ControlOutput>(m, "ControlOutput")
        .def(py::init<double, double>())
        .def_readwrite("ax", &ControlOutput::ax)
        .def_readwrite("omega_dot", &ControlOutput::omega_dot);

    py::class_<VehicleState>(m, "VehicleState")
        .def(py::init<>())
        .def_readwrite("x", &VehicleState::x)
        .def_readwrite("y", &VehicleState::y)
        .def_readwrite("theta", &VehicleState::theta)
        .def_readwrite("v_x", &VehicleState::v_x)
        .def_readwrite("v_y", &VehicleState::v_y)
        .def_readwrite("omega", &VehicleState::omega);

    py::class_<Cone>(m, "Cone")
        .def(py::init<double, double, ConeColor>())
        .def_readwrite("x", &Cone::x)
        .def_readwrite("y", &Cone::y)
        .def_readwrite("c", &Cone::c);

    py::enum_<ConeColor>(m, "ConeColor")
        .value("BLUE", ConeColor::BLUE)
        .value("YELLOW", ConeColor::YELLOW)
        .export_values();

    // export object t (of type Triangulation) to python
    py::class_<Triangulation>(m, "Triangulation")
        .def(py::init<>())
        .def("triangulate", &Triangulation::triangulate)
        .def_readonly("adj", &Triangulation::adj);

    m.def("get_triangulation", &get_triangulation, R"pbdoc(
        Get the current triangulation object)
    )pbdoc");

    m.def("compute", &compute, R"pbdoc(
        Compute control output
    )pbdoc");

    m.def("sim_step", &sim_step, R"pbdoc(
        Simulate one step of the vehicle dynamics
    )pbdoc");
}