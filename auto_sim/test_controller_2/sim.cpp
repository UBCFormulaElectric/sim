#include "sim.hpp"
#include <cmath>
#include <numbers>

void sim_step(VehicleState& state, const ControlOutput& control, const int dt)
{
    const double dt_s = dt / 1000.0;
    state.v_x += (control.ax + state.r * state.v_y) * dt_s;

    const double beta = std::atan2(state.v_y, state.v_x);
    const double tire_ay = -1 * beta; // simple tire model for lateral forces
    state.v_y += (tire_ay - state.r * state.v_x) * dt_s;

    state.psi += std::fmod(state.r * dt_s, 2 * std::numbers::pi);
    const double v_x_world = state.v_x * std::cos(state.psi) - state.v_y * std::sin(state.psi);
    const double v_y_world = state.v_x * std::sin(state.psi) + state.v_y * std::cos(state.psi);
    state.x += v_x_world * dt_s;
    state.y += v_y_world * dt_s;
}