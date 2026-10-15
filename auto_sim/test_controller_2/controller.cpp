#include "controller.hpp"

#include <autodiff/forward/dual.hpp>
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

#include <cassert>
#include <optional>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

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
constexpr static isize n_eq = T * state_dims;
constexpr static isize n_ineq = 0;
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
    Eigen::Matrix<double, state_dims, 1> into() const {
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
static sparse::Vec<double> g(n), b(n_eq), l(n_ineq), u(n_ineq);
static std::optional<sparse::QP<double, isize>> qp_optional = std::nullopt;

static Eigen::Matrix2<autodiff::var> RR(const autodiff::var& theta) {
    const auto l_prime = spline_t_prime(theta);
    return (Eigen::Matrix2<autodiff::var>() << l_prime[0], -l_prime[1], l_prime[1], l_prime[0]).finished();
}
static Eigen::Matrix2<autodiff::var> gamma(const autodiff::var& theta) {
    const static Eigen::Matrix2<autodiff::var> W { { w_1, 0, 0, w_2 } };
    const Eigen::Matrix2<autodiff::var> RR_theta = RR(theta);
    return 1 / spline_t(theta).squaredNorm() * RR_theta.transpose() * W * RR_theta;
}
static autodiff::var cost(const Eigen::Matrix<autodiff::var, state_dims, 1>& state) {
    const Eigen::Vector2<autodiff::var> l_minus_x = spline_t(state(5)) - Eigen::Vector2<autodiff::var>(state(0), state(1));
    return autodiff::var(l_minus_x.transpose() * gamma(state(5)) * l_minus_x) - autodiff::var(w_3 * state(5));
}

void init() {
    // initialize the H, A, C matrices (memory wise I imagine)
    sparse::SparseMat<bool, isize> H_mask(n, n), A_mask(n_eq, n), C_mask(n_ineq, n);
    // TODO populate the masks
    qp_optional = std::make_optional<sparse::QP<double, isize>>(H_mask, A_mask, C_mask);
}

sparse::Vec<double> compute(const VehicleState& ve) {
    // t.adj will contain the adj list
    if (not qp_optional.has_value()) {
        init();
    }
    assert(qp_optional.has_value());
    sparse::QP<double, isize>& qp = qp_optional.value();

    {
        // suppose we have
        const Eigen::Matrix<double, state_dims, 1> x_i;
        Eigen::Matrix<autodiff::var, state_dims, 1> x_i_var;
        // calculate
        Eigen::Matrix<double, state_dims, 1> grad_f;
        const Eigen::Matrix<double, state_dims, state_dims> h = autodiff::hessian(cost(x_i_var), x_i_var, grad_f);
        // construct
        const Eigen::Matrix<double, state_dims, state_dims> P_i = 1 / 2 * h;
        const Eigen::Matrix<double, state_dims, 1> q_i = grad_f - h * x_i;
    }

    // populate H, g, A, b
    qp.update(H, g, A, b, C, l, u);
    // solve
    qp.solve();
    return qp.results.x;
}

ControlOutput get_control(const sparse::Vec<double>& trajectory_control) {
    return {
        trajectory_control[state_dims + state_dims], // u_1[0] = a_x_1
        trajectory_control[state_dims + state_dims + 1], // u_1[1] = r_dot_1
        trajectory_control[state_dims + state_dims + 2], // u_1[2] = delta_1
    };
}

std::array<VehicleState, T> get_trajectory(const sparse::Vec<double>& trajectory_control) {
    std::array<VehicleState, T> out;
    for (isize i = 0; i < T; i++) {
        out[i] = {
            trajectory_control[state_dims + i * dims],
            trajectory_control[state_dims + i * dims + 1],
            trajectory_control[state_dims + i * dims + 2],
            trajectory_control[state_dims + i * dims + 3],
            trajectory_control[state_dims + i * dims + 4],
            trajectory_control[state_dims + i * dims + 5],
        };
    }
    return out;
}
