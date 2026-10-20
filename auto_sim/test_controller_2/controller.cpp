#include "controller.hpp"

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>

#include "path.hpp"
#include "vehicle_model.hpp"

using namespace proxsuite::proxqp;

namespace {
// parameters
constexpr double deltaT_s = 0.1; // time represented per step
constexpr isize state_dims = vehicle_model::state_dims; // state dimensions (x,y,v_x,v_y,psi,r,theta)
constexpr isize control_dims = vehicle_model::control_dims; // control dimensions (a_x,r_dot,delta,v_theta)
constexpr isize dims = state_dims + control_dims;
constexpr double w_1 = 1.0; // TODO tune
constexpr double w_2 = 1.0; // TODO tune
constexpr double w_3 = 3.0; // progress reward. TODO tune (above ~10 the car runs off the track at the first corner)
// control effort weights, these also keep the QP strictly convex in the controls. TODO tune
constexpr double w_a_x = 0.01;
constexpr double w_r_dot = 0.01;
constexpr double w_delta = 70.0; // below ~50 the steering oscillates in closed loop, above this the car starts to understeer
constexpr double w_v_theta = 0.01;

template <typename T>
using StateVec = Eigen::Matrix<T, state_dims, 1>;
template <typename T>
using ControlVec = Eigen::Matrix<T, control_dims, 1>;

// dimensions
constexpr isize n = T * dims;
constexpr isize n_eq = T * state_dims; // model over the state variables
constexpr isize n_ineq = T * control_dims; // only have constraints over the control inputs
struct MPCState {
    double x;
    double y;
    double v_x;
    double v_y;
    double psi;
    double r;
    double theta;
    // theta is the vehicle's progress along the center line, i.e. the projection of (x, y) onto it
    static MPCState from(const VehicleState& state, const double theta) {
        return MPCState {
            .x = state.x,
            .y = state.y,
            .v_x = state.v_x,
            .v_y = state.v_y,
            .psi = state.psi,
            .r = state.r,
            .theta = theta,
        };
    }
    [[nodiscard]] StateVec<double> into() const {
        return { x, y, v_x, v_y, psi, r, theta };
    }
};
static_assert(sizeof(MPCState) == state_dims * sizeof(double), "MPCState size mismatch");
struct MPCControl {
    double a_x;
    double r_dot;
    double delta;
    double v_theta;
};
static_assert(sizeof(MPCControl) == control_dims * sizeof(double), "MPCControl size mismatch");
struct MPCParams {
    double x;
    double y;
    double v_x;
    double v_y;
    double psi;
    double r;
    double theta;
    double a_x;
    double r_dot;
    double delta;
    double v_theta;

    MPCParams() = default;
    MPCParams(const MPCState& s, const MPCControl& c)
        : x(s.x), y(s.y), v_x(s.v_x), v_y(s.v_y), psi(s.psi), r(s.r), theta(s.theta), a_x(c.a_x), r_dot(c.r_dot), delta(c.delta), v_theta(c.v_theta) { }
};
static_assert(sizeof(MPCParams) == dims * sizeof(double), "MPCParams size mismatch");

// matrices
sparse::SparseMat<double, isize>
    H(n, n), A(n_eq, n), C(n_ineq, n);
dense::Vec<double> g(n), b(n_eq), lower(n_ineq), upper(n_ineq);
std::optional<sparse::QP<double, isize>> qp_optional = std::nullopt;

Eigen::Matrix2<autodiff::dual2nd> RR(const autodiff::dual2nd& theta) {
    const Eigen::Vector2<autodiff::dual2nd> l_prime = spline_t_prime(theta);
    return (Eigen::Matrix2<autodiff::dual2nd>() << l_prime[0], -l_prime[1], l_prime[1], l_prime[0]).finished();
}
Eigen::Matrix2<autodiff::dual2nd> gamma(const autodiff::dual2nd& theta) {
    const static Eigen::Matrix2<autodiff::dual2nd> W { { w_1, 0 }, { 0, w_2 } };
    const Eigen::Matrix2<autodiff::dual2nd> RR_theta = RR(theta);
    return 1 / spline_t_prime(theta).squaredNorm() * RR_theta.transpose() * W * RR_theta;
}
autodiff::dual2nd cost(const StateVec<autodiff::dual2nd>& state) {
    const autodiff::dual2nd& theta = state(6);
    const Eigen::Vector2<autodiff::dual2nd> l_minus_x = spline_t(theta) - Eigen::Vector2<autodiff::dual2nd>(state(0), state(1));
    // the progress reward (w_3) is applied to the final step only, in compute()
    return autodiff::dual2nd(l_minus_x.transpose() * gamma(theta) * l_minus_x);
}

constexpr double A_X_MIN = -10.0;
constexpr double A_X_MAX = 1.0;
constexpr double R_DOT_MIN = -1.0;
constexpr double R_DOT_MAX = 1.0;
constexpr double DELTA_MIN = -0.5;
constexpr double DELTA_MAX = 0.5;
constexpr double V_THETA_MIN = -20.0;
constexpr double V_THETA_MAX = 20.0;
}

void init() {
    // initialize the H, A, C matrices (memory wise I imagine)
    sparse::SparseMat<bool, isize> H_mask(n, n), A_mask(n_eq, n), C_mask(n_ineq, n);

    {
        std::vector<Eigen::Triplet<bool>> H_triplet;
        for (isize t = 0; t < T; t++) {
            for (isize i = 0; i < state_dims; i++) {
                for (isize j = 0; j < state_dims; j++) {
                    H_triplet.emplace_back(dims * t + i, dims * t + j, true);
                }
            }
            for (isize i = 0; i < control_dims; i++) {
                H_triplet.emplace_back(dims * t + state_dims + i, dims * t + state_dims + i, true);
            }
        }
        H_mask.setFromTriplets(H_triplet.begin(), H_triplet.end());
    }
    {
        std::vector<Eigen::Triplet<bool>> A_triplet;
        for (isize t = 0; t < T; t++) {
            if (t > 0) {
                for (isize i = 0; i < state_dims; i++) {
                    for (isize j = 0; j < dims; j++) {
                        A_triplet.emplace_back(state_dims * t + i, dims * (t - 1) + j, true);
                    }
                }
            }
            for (isize i = 0; i < state_dims; i++) {
                A_triplet.emplace_back(state_dims * t + i, dims * t + i, true);
            }
        }
        A_mask.setFromTriplets(A_triplet.begin(), A_triplet.end());
    }
    {
        std::vector<Eigen::Triplet<bool>> C_triplet;
        for (isize t = 0; t < T; t++) {
            for (isize i = 0; i < control_dims; i++) {
                C_triplet.emplace_back(control_dims * t + i, dims * t + state_dims + i, true);
            }
        }
        C_mask.setFromTriplets(C_triplet.begin(), C_triplet.end());
    }

    qp_optional = std::make_optional<sparse::QP<double, isize>>(H_mask, A_mask, C_mask);
    qp_optional->settings.max_iter = 1000;
    // each outer iteration can otherwise run up to 1500 inner iterations, which made failed solves take seconds
    qp_optional->settings.max_iter_in = 100;
    // consecutive problems are very similar, so start from the previous solution
    qp_optional->settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    H = H_mask.cast<double>();
    A = A_mask.cast<double>();
    C = C_mask.cast<double>();
    // the control cost is constant, so it only needs to be set once (compute() only overwrites the state blocks)
    const ControlVec<double> w_u { w_a_x, w_r_dot, w_delta, w_v_theta };
    for (isize t = 0; t < T; t++) {
        for (isize i = 0; i < control_dims; i++) {
            H.coeffRef(dims * t + state_dims + i, dims * t + state_dims + i) = w_u(i);
        }
    }
    g.setZero();
    b.setZero();
    for (isize t = 0; t < T; t++) {
        lower(t * control_dims) = A_X_MIN; // a_x lower bound
        lower(t * control_dims + 1) = R_DOT_MIN; // r_dot
        lower(t * control_dims + 2) = DELTA_MIN; // delta
        lower(t * control_dims + 3) = V_THETA_MIN; // v_theta
        upper(t * control_dims) = A_X_MAX; // a_x upper bound
        upper(t * control_dims + 1) = R_DOT_MAX; // r_dot
        upper(t * control_dims + 2) = DELTA_MAX; // delta
        upper(t * control_dims + 3) = V_THETA_MAX; // v_theta
        // TODO constraints for tire grip limits
    }

    // populate the identity matrices inside of A
    for (isize t = 1; t < T; t++) {
        for (isize i = 0; i < state_dims; i++) {
            A.coeffRef(state_dims * t + i, dims * t + i) = -1;
        }
    }
}

static Eigen::Matrix<double, T * dims, 1> last_result;
sparse::Vec<double> compute(const VehicleState& ve, const double dt_s) {
    const bool first_call = not qp_optional.has_value();
    if (first_call) {
        init();
    }
    assert(qp_optional.has_value());
    sparse::QP<double, isize>& qp = qp_optional.value();

    if (not first_call) {
        // the previous solution started dt_s ago, so shift it forward in time to line it up with the current state.
        // steps are linearly interpolated, and the final step is held past the end of the previous horizon
        const Eigen::Matrix<double, T * dims, 1> prev = last_result;
        for (isize i = 0; i < T; i++) {
            const double s = std::min(static_cast<double>(i) + std::max(dt_s, 0.0) / deltaT_s, static_cast<double>(T - 1));
            const isize k = std::min(static_cast<isize>(s), T - 2);
            const double frac = s - static_cast<double>(k);
            last_result.segment<dims>(i * dims) = (1 - frac) * prev.segment<dims>(k * dims) + frac * prev.segment<dims>((k + 1) * dims);
        }
    }

    double theta = 0;
    if (first_call) {
        // there is nothing to seed the projection with, so use the robust (but slow) global search
        theta = project(ve.x, ve.y);
    } else {
        // the shifted previous solution's progress at the current time is a good seed for the fast local search.
        // project() returns theta within a single lap, but theta keeps growing past the end of the center line, so unwrap
        // the projection onto the lap closest to the previous solution to keep the warm start consistent across the seam
        const double theta_prev = last_result(6);
        const double theta_prev_wrapped = theta_prev - center_line_length * std::floor(theta_prev / center_line_length);
        theta = theta_prev + std::remainder(project(ve.x, ve.y, theta_prev_wrapped) - theta_prev_wrapped, center_line_length);
    }

    // populuate b vector with vehicle state
    const StateVec<double> ve_m = MPCState::from(ve, theta).into();
    b.segment(0, state_dims) = ve_m;

    if (first_call) {
        // there is no previous solution to linearize around yet, so hold the current state across the horizon.
        // (an all zero seed puts v=0 into the dynamics, which makes the jacobians NaN)
        for (isize i = 0; i < T; i++) {
            last_result.segment<state_dims>(i * dims) = ve_m;
            last_result.segment<control_dims>(i * dims + state_dims).setZero();
        }
    }

    for (isize i = 0; i < T; i++) {
        // linearize around the (time shifted) previous solution
        StateVec<double> x_i = last_result.segment<state_dims>(i * dims);

        // calculate
        StateVec<double> grad_f { };
        Eigen::Matrix<double, state_dims, state_dims> H_x_i { };
        {
            StateVec<autodiff::dual2nd> x_i_d = x_i.cast<autodiff::dual2nd>();
            autodiff::dual2nd f_x;
            autodiff::hessian(cost, autodiff::wrt(x_i_d), autodiff::at(x_i_d), f_x, grad_f, H_x_i);
        }
        // construct the second order taylor expansion of the cost around x_i.
        // proxsuite minimizes 1/2 x^T H x + g^T x, so the hessian goes in directly (no extra factor of 1/2).
        // the contouring cost is not convex in theta, so project the hessian onto the PSD cone since proxsuite requires a convex QP
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, state_dims, state_dims>> eig(H_x_i);
        const Eigen::Matrix<double, state_dims, state_dims> P_i = eig.eigenvectors()
            * eig.eigenvalues().cwiseMax(0.0).asDiagonal()
            * eig.eigenvectors().transpose();
        // keep the model's gradient at x_i equal to grad_f
        const StateVec<double> q_i = grad_f - P_i * x_i;

        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims; k++) {
                H.coeffRef(i * dims + j, i * dims + k) = P_i(j, k);
            }
        }
        for (isize j = 0; j < state_dims; j++) {
            g.coeffRef(i * dims + j) = q_i(j);
        }

        if (i < 1) {
            continue;
        }

        // populate A, b with the step i-1 -> i dynamics, so linearize around the previous solution at step i-1
        const StateVec<double> x_prev = last_result.segment<state_dims>((i - 1) * dims);
        const ControlVec<double> u_prev = last_result.segment<control_dims>((i - 1) * dims + state_dims);
        StateVec<autodiff::dual> x_prev_dual;
        ControlVec<autodiff::dual> u_prev_dual;
        for (isize j = 0; j < state_dims; j++) {
            x_prev_dual(j) = x_prev(j);
        }
        for (isize j = 0; j < control_dims; j++) {
            u_prev_dual(j) = u_prev(j);
        }

        StateVec<autodiff::dual> xi_prime_at;
        const Eigen::Matrix<double, state_dims, state_dims> A_c = autodiff::jacobian(vehicle_model::xi_prime<autodiff::dual>, autodiff::wrt(x_prev_dual), autodiff::at(x_prev_dual, u_prev_dual), xi_prime_at);
        const Eigen::Matrix<double, state_dims, control_dims> B_c = autodiff::jacobian(vehicle_model::xi_prime<autodiff::dual>, autodiff::wrt(u_prev_dual), autodiff::at(x_prev_dual, u_prev_dual), xi_prime_at);

        const StateVec<double> o = vehicle_model::xi_prime(x_prev_dual, u_prev_dual).cast<double>();
        const StateVec<double> r = o - A_c * x_prev - B_c * u_prev;

        static constexpr isize ee_s = state_dims + control_dims + 1;
        // must be explicitly zeroed (Eigen's {} leaves it uninitialized): the bottom rows have to be 0 for exp() to give the ZOH discretization
        Eigen::Matrix<double, ee_s, ee_s> ee = Eigen::Matrix<double, ee_s, ee_s>::Zero();
        // populate ee from A_c
        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims; k++) {
                ee.coeffRef(j, k) = A_c(j, k);
            }
        }
        // populate ee from B_c
        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < control_dims; k++) {
                ee.coeffRef(j, state_dims + k) = B_c(j, k);
            }
        }
        for (isize j = 0; j < state_dims; j++) {
            ee.coeffRef(j, dims) = r(j);
        }
        ee *= deltaT_s;
        ee = ee.exp();

        // populate the A matrix with ee
        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims + control_dims; k++) {
                A.coeffRef(i * state_dims + j, (i - 1) * dims + k) = ee(j, k);
            }
        }

        // populate the b vector with ee
        for (isize j = 0; j < state_dims; j++) {
            b.coeffRef(i * state_dims + j) = -ee(j, dims);
        }
    }

    // reward the progress made by the end of the horizon. rewarding theta at every step instead counts early progress
    // once per remaining step, which makes the pull towards speed grow with T^2 while the tracking costs only grow with T
    g.coeffRef((T - 1) * dims + 6) -= w_3;

    // populate H, g, A, b
    qp.update(H, g, A, b, C, lower, upper);
    // solve
    qp.solve();
    if (qp.results.info.status != QPSolverOutput::PROXQP_SOLVED) {
        // keep the previous seed rather than linearizing the next call around a bad solution
        std::cerr << "MPC QP did not converge: status " << static_cast<int>(qp.results.info.status)
                  << " after " << qp.results.info.iter << " iterations" << std::endl;
    } else {
        // use qp.results.x to seed the next iteration
        last_result = qp.results.x.segment(0, T * dims);
    }

    return qp.results.x;
}

ControlOutput get_control(const sparse::Vec<double>& trajectory_control) {
    return ControlOutput {
        trajectory_control[state_dims], // u_1[0] = a_x_1
        trajectory_control[state_dims + 1], // u_1[1] = r_dot_1
        trajectory_control[state_dims + 2], // u_1[2] = delta_1
    };
}

std::array<VehicleState, T> get_trajectory(const sparse::Vec<double>& trajectory_control) {
    std::array<VehicleState, T> out;
    for (isize i = 0; i < T; i++) {
        // VehicleState is (x, y, psi, v_x, v_y, r) but the MPC state is (x, y, v_x, v_y, psi, r, theta)
        out[i] = VehicleState {
            trajectory_control[i * dims],
            trajectory_control[i * dims + 1],
            trajectory_control[i * dims + 4],
            trajectory_control[i * dims + 2],
            trajectory_control[i * dims + 3],
            trajectory_control[i * dims + 5],
        };
    }
    return out;
}
