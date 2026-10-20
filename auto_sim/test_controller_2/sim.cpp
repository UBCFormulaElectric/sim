#include "sim.hpp"
#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace {
// the tire dynamics have time constants of a few ms at low speed, so integrate in substeps no longer than this
constexpr double MAX_SUBSTEP_S = 0.001;
}

void sim_step(VehicleState& state, const ControlOutput& control, const int dt)
{
    using vehicle_model::ControlVec;
    using vehicle_model::StateVec;
    using vehicle_model::xi_prime;

    // integrate the same vehicle model the MPC plans with, using RK4.
    // theta (progress along the path) only exists in the controller, so it is carried along as 0 and v_theta is unused
    StateVec<double> x { state.x, state.y, state.v_x, state.v_y, state.psi, state.r, 0.0 };
    const ControlVec<double> u { control.ax, control.omega_dot, control.delta, 0.0 };
    const double dt_s = dt / 1000.0;
    const int substeps = std::max(1, static_cast<int>(std::ceil(dt_s / MAX_SUBSTEP_S)));
    const double h = dt_s / substeps;
    for (int i = 0; i < substeps; i++) {
        const StateVec<double> k1 = xi_prime<double>(x, u);
        const StateVec<double> k2 = xi_prime<double>(x + h / 2 * k1, u);
        const StateVec<double> k3 = xi_prime<double>(x + h / 2 * k2, u);
        const StateVec<double> k4 = xi_prime<double>(x + h * k3, u);
        x += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    state = VehicleState(x(0), x(1), x(4), x(2), x(3), x(5));
}
