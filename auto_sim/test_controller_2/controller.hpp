#pragma once
#include "types.hpp"

#include <optional>
#include <proxsuite/proxqp/sparse/sparse.hpp>

constexpr inline proxsuite::linalg::veg::isize T = 25; // timesteps

struct Parameters {
    double deltaT_s = 0.1; // time represented per step
    double w_1 = 1.0; // TODO tune
    double w_2 = 3.0; // TODO tune
    double w_3 = 7.0; // progress reward. TODO tune (above ~10 the car runs off the track at the first corner)
    double w_a_x = 0.01;
    double w_r_dot = 0.01;
    double w_delta = 10.0; // below ~50 the steering oscillates in closed loop, above this the car starts to understeer
    double w_v_theta = 0.01;
    // trust region: pulls the steering towards the previous solution it was linearized around. without it, a plan that
    // saturates the tires is linearized where steering has no effect, and the next plan flips to the opposite extreme
    double w_prox = 0.0;
    double A_X_MIN = -1.0;
    double A_X_MAX = 1.0;
    double R_DOT_MIN = -0.0;
    double R_DOT_MAX = 0.0;
    double DELTA_MIN = -0.5;
    double DELTA_MAX = 0.5;
    double V_THETA_MIN = -20.0;
    double V_THETA_MAX = 20.0;
};

void init(const std::optional<Parameters> pp = std::nullopt);

/**
 * @param ve
 * @param dt_s seconds elapsed since the previous call, used to shift the previous solution forward in time to warm start
 * this one. ignored on the first call.
 * @return The optimal trajectory from the current vehicle state to the goal state, as computed by the MPC controller.
 */
proxsuite::proxqp::sparse::Vec<double> compute(const VehicleState& ve, double dt_s);

// extract just the next control output from the trajectory control vector
ControlOutput get_control(const proxsuite::proxqp::sparse::Vec<double>& trajectory_control);
// extract the entire predicted trajectory of vehicle states from the trajectory control vector
std::array<VehicleState, T> get_trajectory(const proxsuite::proxqp::sparse::Vec<double>& trajectory_control);

enum class DriverlessState {
    IDLE,
    ONLINE_MAPPING, // use for autocross, acceleration, skidpad, etc.
    TRACKDRIVE_MAPPING,
    TRACKDRIVE_RUNNING
};