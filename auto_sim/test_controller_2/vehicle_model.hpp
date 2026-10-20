#pragma once

#include "constants.hpp"
#include "tire_model.hpp"

#include <Eigen/Dense>
#include <cmath>

/**
 * Vehicle dynamics shared by the MPC (which differentiates them with T = autodiff::dual) and the simulator (T = double).
 */
namespace vehicle_model {
constexpr double m = CAR_MASS_AT_CG_KG; // mass of the vehicle
constexpr double izz = CAR_YAW_MOMENT_INERTIA_KGM2; // moment of inertia about the z-axis

constexpr int state_dims = 7; // state dimensions (x,y,v_x,v_y,psi,r,theta)
constexpr int control_dims = 4; // control dimensions (a_x,r_dot,delta,v_theta)

template <typename T>
using StateVec = Eigen::Matrix<T, state_dims, 1>;
template <typename T>
using ControlVec = Eigen::Matrix<T, control_dims, 1>;

// slip angles (front, rear)
template <typename T>
Eigen::Matrix<T, 2, 1> alpha(const T& delta, const T& vx, const T& vy, const T& r) {
    using std::atan2;
    // floor v_x so the slip angle (and its derivative, which is 0/0 at v=0) stays finite near standstill
    const T vx_reg = vx < SLIP_REGULARIZATION_SPEED_MPS ? T(SLIP_REGULARIZATION_SPEED_MPS) : vx;
    // the lateral velocity at each axle includes the yaw rate's contribution, which is what damps the yaw motion
    return {
        delta - atan2(vy + DIST_FRONT_AXLE_CG_m * r, vx_reg),
        -atan2(vy - DIST_REAR_AXLE_CG_m * r, vx_reg),
    };
}

// lateral tire forces (front left, front right, rear left, rear right)
template <typename T>
Eigen::Matrix<T, 4, 1> F_y(const T& delta, const T& vx, const T& vy, const T& r) {
    const Eigen::Matrix<T, 2, 1> alpha_i = alpha(delta, vx, vy, r);
    constexpr double f_z = 890.0; // TODO compute from vehicle state
    return {
        tire_model.computeCombinedFy_N(f_z, alpha_i(0), T(0.0)),
        tire_model.computeCombinedFy_N(f_z, alpha_i(0), T(0.0)),
        tire_model.computeCombinedFy_N(f_z, alpha_i(1), T(0.0)),
        tire_model.computeCombinedFy_N(f_z, alpha_i(1), T(0.0)),
    };
}

// yaw moment about the CG from the lateral tire forces (front left, front right, rear left, rear right)
template <typename T>
T M_z(const Eigen::Matrix<T, 4, 1>& f_y) {
    return DIST_FRONT_AXLE_CG_m * (f_y(0) + f_y(1)) - DIST_REAR_AXLE_CG_m * (f_y(2) + f_y(3));
}

// time derivative of the state
template <typename T>
StateVec<T> xi_prime(const StateVec<T>& x, const ControlVec<T>& u) {
    using std::cos;
    using std::sin;
    // x=(x,y,v_x,v_y,psi,r,theta)
    // u=(a_x,r_dot,delta,v_theta)
    const Eigen::Matrix<T, 4, 1> F_y_i = F_y(u(2), x(2), x(3), x(5));
    StateVec<T> out { };
    out << x(2) * cos(x(4)) - x(3) * sin(x(4)), // x_dot: body frame velocity rotated into the global frame
        x(2) * sin(x(4)) + x(3) * cos(x(4)), // y_dot
        x(5) * x(3) + u(0), // v_x_dot
        1 / m * F_y_i.sum() - x(5) * x(2), // v_y_dot
        x(5), // psi_dot
        1 / izz * M_z(F_y_i) + u(1), // r_dot
        u(3); // theta_dot
    return out;
}
}
