#pragma once

#include "constants.hpp"
#include "tire_model.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>

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

// normal loads (front left, front right, rear left, rear right): static weight, aero downforce, and longitudinal and
// lateral load transfer
template <typename T>
Eigen::Matrix<T, 4, 1> F_z(const T& vx, const T& r, const T& a_x) {
    constexpr double g = GRAVITY;
    constexpr double l = WHEELBASE_m;
    constexpr double l_f = DIST_FRONT_AXLE_CG_m;
    constexpr double l_r = DIST_REAR_AXLE_CG_m;
    constexpr double h = DIST_HEIGHT_CG_m;
    constexpr double t = TRACK_WIDTH_m;
    constexpr double f_static_front = m * g * l_r / l / 2; // per wheel
    constexpr double f_static_rear = m * g * l_f / l / 2;

    const T f_aero = 0.5 * AIR_DENSITY_KGPM3 * LIFT_COEFF * FRONTAL_AREA_M2 * vx * vx;
    // a_x is the body frame longitudinal acceleration, which shifts load from the front axle to the rear
    const T df_long = m * h / (2 * l) * a_x; // per wheel
    // steady state body frame lateral acceleration (ignores v_y_dot, which would make F_z depend on F_y). each axle
    // takes the share of the transfer it carries statically, since there is no roll stiffness distribution to split by
    const T a_y = r * vx;
    const T df_lat_front = m * l_r / l * h / t * a_y; // moved from the left wheel to the right
    const T df_lat_rear = m * l_f / l * h / t * a_y;
    return {
        f_static_front - df_long - df_lat_front + (1 - COP_REAR) * (1 - COP_RIGHT) * f_aero,
        f_static_front - df_long + df_lat_front + (1 - COP_REAR) * COP_RIGHT * f_aero,
        f_static_rear + df_long - df_lat_rear + COP_REAR * (1 - COP_RIGHT) * f_aero,
        f_static_rear + df_long + df_lat_rear + COP_REAR * COP_RIGHT * f_aero,
    };
}

// lateral force of one tire
template <typename T>
T F_y_tire(const T& f_z, const T& alpha) {
    if constexpr (std::is_arithmetic_v<T>) {
        return tire_model.computeCombinedFy_N(static_cast<float>(f_z), alpha, T(0.0));
    } else {
        // the tire model only takes F_z as a float, so differentiate w.r.t. F_z by central difference and chain it
        // onto the dual by hand. otherwise the MPC's linearization would miss how load transfer changes grip
        const double f_z0 = autodiff::val(f_z);
        const double alpha0 = autodiff::val(alpha);
        constexpr double h = 10.0;
        const double dfy_dfz = (tire_model.computeCombinedFy_N(static_cast<float>(f_z0 + h), alpha0, 0.0)
                                   - tire_model.computeCombinedFy_N(static_cast<float>(f_z0 - h), alpha0, 0.0))
            / (2 * h);
        return tire_model.computeCombinedFy_N(static_cast<float>(f_z0), alpha, T(0.0)) + dfy_dfz * (f_z - f_z0);
    }
}

// lateral tire forces (front left, front right, rear left, rear right)
template <typename T>
Eigen::Matrix<T, 4, 1> F_y(const T& delta, const T& vx, const T& vy, const T& r, const Eigen::Matrix<T, 4, 1>& f_z) {
    const Eigen::Matrix<T, 2, 1> alpha_i = alpha(delta, vx, vy, r);
    return {
        F_y_tire(f_z(0), alpha_i(0)),
        F_y_tire(f_z(1), alpha_i(0)),
        F_y_tire(f_z(2), alpha_i(1)),
        F_y_tire(f_z(3), alpha_i(1)),
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
    const Eigen::Matrix<T, 4, 1> F_z_i = F_z(x(2), x(5), u(0));
    const Eigen::Matrix<T, 4, 1> F_y_i = F_y(u(2), x(2), x(3), x(5), F_z_i);
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
