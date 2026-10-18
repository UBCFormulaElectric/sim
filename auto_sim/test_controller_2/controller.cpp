#include "controller.hpp"

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cassert>
#include <optional>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "path.hpp"

using namespace proxsuite::proxqp;

// parameters
constexpr static double deltaT_s = 0.1; // time represented per step
constexpr static isize state_dims = 6; // state dimensions (x,y,v_x,v_y,r,theta)
constexpr static isize control_dims = 4; // control dimensions (a_x,r_dot,delta,v_theta)
constexpr static isize dims = state_dims + control_dims;
constexpr static double w_1 = 1.0; // TODO tune
constexpr static double w_2 = 1.0; // TODO tune
constexpr static double w_3 = 1.0; // TODO tune

// dimensions
constexpr static isize n = T * dims;
constexpr static isize n_eq = T * state_dims; // model over the state variables
constexpr static isize n_ineq = T * control_dims; // only have constraints over the control inputs
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
        return { state.x, state.y, state.v_x, state.v_y, state.psi, state.r, 0 };
    }
    [[nodiscard]] Eigen::Matrix<double, state_dims, 1> into() const {
        return { x, y, v_x, v_y, psi, r };
    }
};
struct MPCControl {
    double a_x;
    double r_dot;
    double delta;
    double v_theta;
};
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

// matrices
static sparse::SparseMat<double, isize>
    H(n, n), A(n_eq, n), C(n_ineq, n);
static dense::Vec<double> g(n), b(n_eq), lower(n_ineq), upper(n_ineq);
static std::optional<sparse::QP<double, isize>> qp_optional = std::nullopt;

static Eigen::Matrix2<autodiff::dual2nd> RR(const autodiff::dual2nd& theta) {
    const auto l_prime = spline_t_prime(theta);
    return (Eigen::Matrix2<autodiff::dual2nd>() << l_prime[0], -l_prime[1], l_prime[1], l_prime[0]).finished();
}
static Eigen::Matrix2<autodiff::dual2nd> gamma(const autodiff::dual2nd& theta) {
    const static Eigen::Matrix2<autodiff::dual2nd> W { { w_1, 0, 0, w_2 } };
    const Eigen::Matrix2<autodiff::dual2nd> RR_theta = RR(theta);
    return 1 / spline_t(theta).squaredNorm() * RR_theta.transpose() * W * RR_theta;
}
static autodiff::dual2nd cost(const Eigen::Matrix<autodiff::dual2nd, state_dims, 1>& state) {
    const Eigen::Vector2<autodiff::dual2nd> l_minus_x = spline_t(state(5)) - Eigen::Vector2<autodiff::dual2nd>(state(0), state(1));
    return autodiff::dual2nd(l_minus_x.transpose() * gamma(state(5)) * l_minus_x)
        - autodiff::dual2nd(w_3) * state(5);
}

static Eigen::Matrix<autodiff::dual, state_dims, 1> xi_prime(const Eigen::Matrix<autodiff::dual, state_dims, 1>& x, const Eigen::Matrix<autodiff::dual, control_dims, 1>& u) {
    // TODO implement the process model
    return { };
};

void init() {
    // initialize the H, A, C matrices (memory wise I imagine)
    sparse::SparseMat<bool, isize> H_mask(n, n), A_mask(n_eq, n), C_mask(n_ineq, n);
    for (isize t = 0; t < T; t++) {
        for (isize i = 0; i < n; i++) {
            for (isize j = 0; j < n; j++) {
                H_mask.coeffRef(state_dims * t + i, state_dims + dims * t + j) = true;
            }
        }
    }
    for (isize t = 0; t < T; t++) {
        for (isize i = 0; i < n_eq; i++) {
            for (isize j = 0; j < n; j++) {
                A_mask.coeffRef(i, j) = true;
            }
        }
        for (isize i = 0; i < state_dims; i++) {
            A_mask.coeffRef(state_dims * t + i, dims * t + i) = true;
        }
    }
    for (isize i = 0; i < n_ineq; i++) {
        C_mask.coeffRef(i, state_dims + i) = true;
    }
    qp_optional = std::make_optional<sparse::QP<double, isize>>(H_mask, A_mask, C_mask);
    H = H_mask.cast<double>();
    A = A_mask.cast<double>();
    C = C_mask.cast<double>();

    // populate the identity matrices inside of A
    for (isize t = 1; t < T; t++) {
        for (isize i = 0; i < state_dims; i++) {
            A.coeffRef(state_dims * t + i, dims * t + i) = -1;
        }
    }
}

sparse::Vec<double> compute(const VehicleState& ve) {
    // t.adj will contain the adj list
    if (not qp_optional.has_value()) {
        init();
    }
    assert(qp_optional.has_value());
    sparse::QP<double, isize>& qp = qp_optional.value();

    static Eigen::Matrix<double, T * dims, 1> last_result;

    for (isize i = 0; i < T; i++) {
        // we use last_result directly implying that the movement between iterations is sufficiently small
        // to not require a new seed for the optimization algorithm
        Eigen::Matrix<double, state_dims, 1> x_i = last_result.segment<state_dims>(i * dims);

        // calculate
        Eigen::Matrix<double, state_dims, 1> grad_f { };
        Eigen::Matrix<double, state_dims, state_dims> H_x_i { };
        {
            Eigen::Matrix<autodiff::dual2nd, state_dims, 1> x_i_d;
            for (isize j = 0; j < state_dims; j++) {
                x_i_d(j) = x_i(j);
            }
            autodiff::dual2nd f_x;
            autodiff::hessian(cost, autodiff::wrt(x_i_d), autodiff::at(x_i_d), f_x, grad_f, H_x_i);
        }
        // construct
        const Eigen::Matrix<double, state_dims, state_dims> P_i = 1 / 2 * H_x_i;
        const Eigen::Matrix<double, state_dims, 1> q_i = grad_f - H_x_i * x_i;

        for (isize j = 0; j < state_dims; j++) {
            for (isize k = 0; k < state_dims; k++) {
                H.coeffRef(i * state_dims + j, state_dims + i * state_dims + k) = P_i(j, k);
            }
        }
        for (isize j = 0; j < state_dims; j++) {
            g.coeffRef(state_dims + i * state_dims + j) = q_i(j);
        }
    }

    // populuate b vector with vehicle state
    const Eigen::Matrix<double, state_dims, 1> ve_m = MPCState::from(ve).into();
    for (isize j = 0; j < state_dims; j++) {
        b.coeffRef(j) = ve_m(j);
    }

    for (isize i = 1; i < T; i++) {
        // populate A, b
        Eigen::Matrix<double, state_dims, 1> x_i = last_result.segment<state_dims>(i * dims + 0);
        Eigen::Matrix<double, control_dims, 1> u_i = last_result.segment<control_dims>(i * dims + state_dims);
        Eigen::Matrix<autodiff::dual, state_dims, 1> x_i_dual;
        Eigen::Matrix<autodiff::dual, control_dims, 1> u_i_dual;
        for (isize j = 0; j < state_dims; j++) {
            x_i_dual(j) = x_i(j);
        }
        for (isize j = 0; j < control_dims; j++) {
            u_i_dual(j) = u_i(j);
        }

        Eigen::Matrix<autodiff::dual, state_dims, 1> xi_prime_at;
        const Eigen::Matrix<double, state_dims, state_dims> A_c = autodiff::jacobian(xi_prime, autodiff::wrt(x_i_dual), autodiff::at(x_i_dual, u_i_dual), xi_prime_at);
        const Eigen::Matrix<double, state_dims, control_dims> B_c = autodiff::jacobian(xi_prime, autodiff::wrt(u_i_dual), autodiff::at(x_i_dual, u_i_dual), xi_prime_at);

        const Eigen::Matrix<double, state_dims, 1> o = xi_prime(x_i_dual, u_i_dual).cast<double>();
        const Eigen::Matrix<double, state_dims, 1> r = o - A_c * x_i - B_c * u_i;

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
    last_result = qp.results.x.segment(state_dims, state_dims + T * dims);

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
