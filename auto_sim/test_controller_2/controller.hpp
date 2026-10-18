#pragma once
#include "types.hpp"

#include <proxsuite/proxqp/sparse/sparse.hpp>

constexpr inline proxsuite::linalg::veg::isize T = 10; // timesteps

void init();

/**
 * @param ve
 * @return The optimal trajectory from the current vehicle state to the goal state, as computed by the MPC controller.
 */
proxsuite::proxqp::sparse::Vec<double> compute(const VehicleState& ve);

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