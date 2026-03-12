#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

struct ControlOutput {
	double ax;
	double ay;
    double omega_dot;
    ControlOutput(double _ax, double _ay, double _omega_dot) : ax(_ax), ay(_ay), omega_dot(_omega_dot) {}
};
struct VehicleState {
    double x;
    double y;
    double theta;
    double v_x;
    double v_y;
    double omega;
    VehicleState() : x(0), y(0), theta(0), v_x(0), v_y(0), omega(0) {}
};
enum class ConeColor {
    BLUE,
    YELLOW
};
struct Cone {
    double x;
    double y;
    ConeColor c;
    Cone(double _x, double _y, ConeColor _c) : x(_x), y(_y), c(_c) {}
};

ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones) {
    // Implementation for compute function
    return {0,0,0};
}

namespace py = pybind11;

PYBIND11_MODULE(Controller, m) {
    py::class_<ControlOutput>(m, "ControlOutput")
        .def(py::init<double, double, double>())
        .def_readwrite("ax", &ControlOutput::ax)
        .def_readwrite("ay", &ControlOutput::ay)
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

    m.def("compute", &compute, R"pbdoc(
        Compute control output
    )pbdoc");
}