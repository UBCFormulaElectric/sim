#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "CDT.h"
#include "controller.hpp"
#include "path.hpp"
#include "perception.hpp"
#include "sim.hpp"
#include "types.hpp"

namespace py = pybind11;

static CDT::EdgeUSet& get_offline_edges() {
    return offline_inner_edges;
}
static CDT::EdgeUSet& get_boundary_edges() {
    return offline_boundary_edges;
}
static std::vector<Cone>& get_center_points() {
    return center_points;
}
static std::vector<CDT::VertInd>& get_center_line() {
    return center_line_idxs;
}
static double get_center_line_length() {
    return center_line_length;
}

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

    py::class_<Location>(m, "Location")
        .def(py::init<double, double>())
        .def_readwrite("x", &Location::x)
        .def_readwrite("y", &Location::y);

    m.def("compute", &compute, R"pbdoc(
        Compute control output
    )pbdoc");

    m.def("compute_path", &compute_path_from_percepted_cones, R"pbdoc()
        Compute the triangulation from a list of cones
    )pbdoc");

    // debugging visualization purposes
    m.def("get_offline_edges", &get_offline_edges, py::return_value_policy::reference, R"pbdoc()
        Get the offline edges calculated from compute_path()
    )pbdoc");
    m.def("get_boundary_edges", &get_boundary_edges, py::return_value_policy::reference, R"pbdoc(
        Get the boundary edges calculated from compute_path()
    )pbdoc");
    m.def("get_center_points", &get_center_points, py::return_value_policy::reference, R"pbdoc(
        Get the center points calculated from compute_path()
    )pbdoc");
    m.def("get_center_line", &get_center_line, py::return_value_policy::reference, R"pbdoc(
        Get the center line calculated from compute_path()
    )pbdoc");
    m.def("get_center_line_length", &get_center_line_length, R"pbdoc(
        Get the length of the center line calculated from compute_path()
    )pbdoc");

    m.def("sim_step", &sim_step, R"pbdoc(
        Simulate one step of the vehicle dynamics
    )pbdoc");

    m.def("project", [](const double a, const double b) { return project(a, b); }, R"pbdoc(
        Project a point onto a line defined by two points
    )pbdoc");
    m.def("project_seeded", [](const double a, const double b, const double c) { return project(a, b, c); }, R"pbdoc()
        Project a point onto a line defined by two points, with a seed for the optimization algorithm
    )pbdoc");
    m.def("spline_t", &spline_t, R"pbdoc()
        Get the point on the center line corresponding to a given t
    )pbdoc");

    m.def("mock_perception", &mock_perception, R"pbdoc(
        Mock the perception module by directly setting the list of cones
    )pbdoc");

    static py::exception<alglib::ap_error> ex(m, "Alglib::ApError");
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p)
                std::rethrow_exception(p);
        } catch (const alglib::ap_error& e) {
            // Set the Python error using a custom field instead of .what()
            ex(e.msg.c_str());
        }
    });
}