#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controller.hpp"
#include "path.hpp"
#include "sim.hpp"
#include "types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(Controller, m, py::mod_gil_not_used()) {
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

    py::enum_<ConeColor>(m, "ConeColor")
        .value("BLUE", ConeColor::BLUE)
        .value("YELLOW", ConeColor::YELLOW)
        .export_values();

    py::class_<Cone>(m, "Cone")
        .def(py::init<double, double, ConeColor>())
        .def_readwrite("x", &Cone::x)
        .def_readwrite("y", &Cone::y)
        .def_readwrite("c", &Cone::c);

    py::class_<CDT::Edge>(m, "Edge")
        .def("v1", &CDT::Edge::v1)
        .def("v2", &CDT::Edge::v2);

    m.def("compute", &compute, R"pbdoc(
        Compute control output
    )pbdoc");

    // offline path calculations
    m.def("calculate_boundary", &calculate_boundary, R"pbdoc(
        Calculate the boundary of the given color from the list of cones
    )pbdoc");
    m.def("compute_path", &compute_path, R"pbdoc()
        Compute the triangulation from a list of cones
    )pbdoc");
    m.def("get_offline_edges", &get_offline_edges, py::return_value_policy::reference, R"pbdoc()
        Get the offline edges calculated from compute_path()
    )pbdoc");
    m.def("get_boundary_edges", &get_boundary_edges, py::return_value_policy::reference, R"pbdoc()
        Get the boundary edges calculated from compute_path()
    )pbdoc");
    m.def("get_center_points", &get_center_points, py::return_value_policy::reference, R"pbdoc())
        Get the center points calculated from compute_path()
    )pbdoc");
    m.def("get_center_line", get_center_line, py::return_value_policy::reference, R"pbdoc())
        Get the center line calculated from compute_path()
    )pbdoc");
    m.def("project", &project, R"pbdoc())
        Project a point onto the center line
    )pbdoc");

    // online path caclulations
    m.def("update_cone_positions", &update_cone_positions, R"pbdoc()
        Update the triangulation with new cone positions
    )pbdoc");
    m.def("get_triangulation", &get_triangulation, py::return_value_policy::reference, R"pbdoc(
        Get the current triangulation object)
    )pbdoc");

    m.def("sim_step", &sim_step, R"pbdoc(
        Simulate one step of the vehicle dynamics
    )pbdoc");
}