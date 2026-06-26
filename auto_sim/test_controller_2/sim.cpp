#include "sim.hpp"
#include <cmath>
#include <numbers>

void sim_step(VehicleState& state, const ControlOutput& control, const int dt)
{
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