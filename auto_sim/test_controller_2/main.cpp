#include <cassert>
#include <numbers>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <unordered_map>
#include <vector>

struct ControlOutput {
  // in local frame
  double ax;
  // no frame
  double omega_dot;
  ControlOutput(const double _ax, const double _omega_dot)
      : ax(_ax), omega_dot(_omega_dot) {}
};
struct VehicleState {
  // in global frame
  double x;
  double y;
  double theta;
  // in local frame
  double v_x;
  double v_y;
  // no frame
  double omega;
  VehicleState() : x(0), y(0), theta(0), v_x(0), v_y(0), omega(0) {}
};
enum class ConeColor { BLUE, YELLOW };
struct Cone {
  double x;
  double y;
  ConeColor c;
  Cone(double _x, double _y, ConeColor _c) : x(_x), y(_y), c(_c) {}
};

void sim_step(VehicleState &state, const ControlOutput &control, const int dt) {
  const double dt_s = dt / 1000.0;
  state.v_x += (control.ax + state.omega * state.v_y) * dt_s;

  const double beta = std::atan2(state.v_y, state.v_x);
  const double tire_ay = -1 * beta; // simple tire model for lateral forces
  state.v_y += (tire_ay - state.omega * state.v_x) * dt_s;

  state.theta += std::fmod(state.omega * dt_s, 2 * std::numbers::pi);
  const double v_x_world = state.v_x * std::cos(state.theta) - state.v_y * std::sin(state.theta);
  const double v_y_world = state.v_x * std::sin(state.theta) + state.v_y * std::cos(state.theta);
  state.x += v_x_world * dt_s;
  state.y += v_y_world * dt_s;
}

static std::unordered_map<int, std::vector<int>> adj{};
/**
 * Triangulates the cones.
 * @param cones The vector of cones to triangulate.
 * @note This function assumes that the input vector of cones is an add-only
 * array whose order is preserved.
 */
void triangulate(const std::vector<Cone> &cones) {
  static size_t last_seen_cone = 0;
  for (; last_seen_cone < cones.size(); ++last_seen_cone) {
    const Cone &cone = cones[last_seen_cone];
    // process the new cone
  }
}

ControlOutput compute(const VehicleState &ve, const std::vector<Cone> &cones) {
  // Implementation for compute function
  triangulate(cones);
  return {0, 0};
}

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

  m.def("sim_step", &sim_step, R"pbdoc(
        Simulate one step of the vehicle dynamics
    )pbdoc");
}