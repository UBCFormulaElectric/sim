#include "controller.hpp"
#include <cassert>
#include <optional>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

using namespace proxsuite::proxqp;

// parameters
constexpr static double deltaT_s = 0.1; // time represented per step
constexpr static isize state_dims = 6; // state dimensions (x,y,v_x,v_y,r,theta)
constexpr static isize control_dims = 4; // control dimensions (a_x,r_dot,delta,v_theta)
constexpr static isize dims = state_dims + control_dims;

// dimensions
constexpr static isize n = T * dims;
constexpr static isize n_eq = T * state_dims;
constexpr static isize n_ineq = 0;

// matrices
static sparse::SparseMat<double, isize> H(n, n), A(n_eq, n), C(n_ineq, n);
static sparse::Vec<double> g(n), b(n_eq), l(n_ineq), u(n_ineq);
static std::optional<sparse::QP<double, isize>> qp_optional = std::nullopt;

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

    // populate H, g, A, b

    // solve
    qp.update(H, g, A, b, C, l, u);
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
