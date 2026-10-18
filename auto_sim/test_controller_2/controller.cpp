#include "controller.hpp"

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cassert>
#include <optional>

#include "path.hpp"

using namespace proxsuite::proxqp;

namespace {
// parameters
constexpr double deltaT_s = 0.1; // time represented per step
constexpr isize state_dims = 7; // state dimensions (x,y,v_x,v_y,psi,r,theta)
constexpr isize control_dims = 4; // control dimensions (a_x,r_dot,delta,v_theta)
constexpr isize dims = state_dims + control_dims;
constexpr double w_1 = 1.0; // TODO tune
constexpr double w_2 = 1.0; // TODO tune
constexpr double w_3 = 1.0; // TODO tune

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
    static MPCState from(const VehicleState& state) {
        // TODO how to seed theta?
        return MPCState {
            .x = state.x,
            .y = state.y,
            .v_x = state.v_x,
            .v_y = state.v_y,
            .psi = state.psi,
            .r = state.r,
            .theta = 0,
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
    const auto l_prime = spline_t_prime(theta);
    return (Eigen::Matrix2<autodiff::dual2nd>() << l_prime[0], -l_prime[1], l_prime[1], l_prime[0]).finished();
}
Eigen::Matrix2<autodiff::dual2nd> gamma(const autodiff::dual2nd& theta) {
    const static Eigen::Matrix2<autodiff::dual2nd> W { { w_1, 0, 0, w_2 } };
    const Eigen::Matrix2<autodiff::dual2nd> RR_theta = RR(theta);
    return 1 / spline_t(theta).squaredNorm() * RR_theta.transpose() * W * RR_theta;
}
autodiff::dual2nd cost(const StateVec<autodiff::dual2nd>& state) {
    const Eigen::Vector2<autodiff::dual2nd> l_minus_x = spline_t(state(5)) - Eigen::Vector2<autodiff::dual2nd>(state(0), state(1));
    return autodiff::dual2nd(l_minus_x.transpose() * gamma(state(5)) * l_minus_x)
        - autodiff::dual2nd(w_3) * state(5);
}

StateVec<autodiff::dual> xi_prime(const StateVec<autodiff::dual>& x, const ControlVec<autodiff::dual>& u) {
    // TODO implement the process model
    return { };
}
}

void init() {
    // initialize the H, A, C matrices (memory wise I imagine)
    sparse::SparseMat<bool, isize> H_mask(n, n), A_mask(n_eq, n), C_mask(n_ineq, n);

    {
        std::vector<Eigen::Triplet<bool>> H_triplet;
        for (isize t = 0; t < T; t++) {
            for (isize i = 0; i < n; i++) {
                for (isize j = 0; j < n; j++) {
                    H_triplet.emplace_back(state_dims * t + i, state_dims + dims * t + j, true);
                }
            }
        }
        H_mask.setFromTriplets(H_triplet.begin(), H_triplet.end());
    }
    {
        std::vector<Eigen::Triplet<bool>> A_triplet;
        for (isize t = 0; t < T; t++) {
            for (isize i = 0; i < n_eq; i++) {
                for (isize j = 0; j < n; j++) {
                    A_triplet.emplace_back(state_dims * t + i, dims * t + j, true);
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
        for (isize i = 0; i < n_ineq; i++) {
            C_triplet.emplace_back(i, state_dims + i, true);
        }
        C_mask.setFromTriplets(C_triplet.begin(), C_triplet.end());
    }

    qp_optional = std::make_optional<sparse::QP<double, isize>>(H_mask, A_mask, C_mask);
    H = H_mask.cast<double>();
    A = A_mask.cast<double>();
    C = C_mask.cast<double>();
    // TODO populate constraints lower and upper

    // populate the identity matrices inside of A
    for (isize t = 1; t < T; t++) {
        for (isize i = 0; i < state_dims; i++) {
            A.coeffRef(state_dims * t + i, dims * t + i) = -1;
        }
    }
}

static Eigen::Matrix<double, T * dims, 1> last_result;
sparse::Vec<double> compute(const VehicleState& ve) {
    // t.adj will contain the adj list
    if (not qp_optional.has_value()) {
        init();
    }
    assert(qp_optional.has_value());
    sparse::QP<double, isize>& qp = qp_optional.value();

    // populuate b vector with vehicle state
    const StateVec<double> ve_m = MPCState::from(ve).into();
    b.segment(0, state_dims) = ve_m;

    for (isize i = 0; i < T; i++) {
        // we use last_result directly implying that the movement between iterations is sufficiently small
        // to not require a new seed for the optimization algorithm
        StateVec<double> x_i = last_result.segment<state_dims>(i * dims);

        // calculate
        StateVec<double> grad_f { };
        Eigen::Matrix<double, state_dims, state_dims> H_x_i { };
        {
            StateVec<autodiff::dual2nd> x_i_d = x_i.cast<autodiff::dual2nd>();
            autodiff::dual2nd f_x;
            autodiff::hessian(cost, autodiff::wrt(x_i_d), autodiff::at(x_i_d), f_x, grad_f, H_x_i);
        }
        // construct
        const Eigen::Matrix<double, state_dims, state_dims> P_i = 1 / 2 * H_x_i;
        const StateVec<double> q_i = grad_f - H_x_i * x_i;

        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims; k++) {
                H.coeffRef(i * state_dims + j, state_dims + i * state_dims + k) = P_i(j, k);
            }
        }
        for (isize j = 0; j < state_dims; j++) {
            g.coeffRef(state_dims + i * state_dims + j) = q_i(j);
        }

        if (i < 1) {
            continue;
        }

        // populate A, b
        ControlVec<double> u_i = last_result.segment<control_dims>(i * dims + state_dims);
        StateVec<autodiff::dual> x_i_dual;
        ControlVec<autodiff::dual> u_i_dual;
        for (isize j = 0; j < state_dims; j++) {
            x_i_dual(j) = x_i(j);
        }
        for (isize j = 0; j < control_dims; j++) {
            u_i_dual(j) = u_i(j);
        }

        StateVec<autodiff::dual> xi_prime_at;
        const Eigen::Matrix<double, state_dims, state_dims> A_c = autodiff::jacobian(xi_prime, autodiff::wrt(x_i_dual), autodiff::at(x_i_dual, u_i_dual), xi_prime_at);
        const Eigen::Matrix<double, state_dims, control_dims> B_c = autodiff::jacobian(xi_prime, autodiff::wrt(u_i_dual), autodiff::at(x_i_dual, u_i_dual), xi_prime_at);

        const StateVec<double> o = xi_prime(x_i_dual, u_i_dual).cast<double>();
        const StateVec<double> r = o - A_c * x_i - B_c * u_i;

        static constexpr isize ee_s = state_dims + control_dims + 1;
        Eigen::Matrix<double, ee_s, ee_s> ee { };
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
        ee = ee.exp();

        // populate the A matrix with ee
        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims + control_dims; k++) {
                A.coeffRef(i * state_dims + j, (i - 1) * dims + k) = ee(j, k);
            }
        }

        // populate the b vector with ee
        for (isize j = 0; j < state_dims; j++) {
            b.coeffRef(i * state_dims + j) += -ee(j, dims);
        }
    }

    // populate H, g, A, b
    qp.update(H, g, A, b, C, lower, upper);
    // solve
    qp.solve();
    // use qp.results.x to seed the next iteration
    last_result = qp.results.x.segment(0, T * dims);

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
        out[i] = VehicleState {
            trajectory_control[i * dims],
            trajectory_control[i * dims + 1],
            trajectory_control[i * dims + 2],
            trajectory_control[i * dims + 3],
            trajectory_control[i * dims + 4],
            trajectory_control[i * dims + 5],
        };
    }
    return out;
}
