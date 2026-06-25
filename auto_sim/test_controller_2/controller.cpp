#include "controller.hpp"
#include <proxsuite/proxqp/dense/dense.hpp>

using namespace proxsuite::proxqp;

ControlOutput compute(const VehicleState& ve) {
    constexpr isize dim = 3, n_eq = 0, n_in = 3;
    constexpr double eps_abs = 1e-9;

    // qp
    Eigen::Matrix<double, dim, dim> H;
    Eigen::Vector<double, dim> g = Eigen::Vector<double, dim>::Random(dim);

    // bounds constraints
    Eigen::Matrix<double, n_in, dim> C;
    Eigen::Vector<double, n_in> l;
    l << -1.0, -1.0, -1.0;
    Eigen::Vector<double, n_in> u;
    u << 1.0, 1.0, 1.0;

    // equality constraints
    Eigen::Matrix<double, n_eq, dim> A;
    Eigen::Vector<double, n_eq> b = Eigen::Vector<double, n_eq>::Zero(n_eq);

    dense::QP<double> qp(dim, n_eq, n_in);
    qp.settings.eps_abs = eps_abs;
    qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    qp.settings.verbose = true;
    qp.init(H, g, A, b, C, l, u);
    qp.solve();

    // std::cout << "primal residual: " << qp.results.info.pri_res << std::endl;
    // std::cout << "dual residual: " << qp.results.info.dua_res << std::endl;
    // std::cout << "total number of iteration: " << qp.results.info.iter
    //           << std::endl;
    // std::cout << "setup timing " << qp.results.info.setup_time << " solve time "
    //           << qp.results.info.solve_time << std::endl;

    return { 0, 0 };
}