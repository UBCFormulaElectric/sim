#include "path.hpp"
#include "perception.hpp"
#include "scopetimer.hpp"
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

// spacing of the resampled center line knots
constexpr double CENTER_LINE_SPACING_M = 0.5;
// wiggles in the center line shorter than about this wavelength are smoothed out. the midpoints of the cone pairs zig-zag
// with a ~2 m period, while the tightest real corner (a ~3 m radius hairpin) spans ~10 m of arc. on labels/0000075 this
// moves the line at most 13 cm from the midpoints and raises the tightest radius from 1.3 m (wiggles) to 3.0 m
constexpr double CENTER_LINE_SMOOTHING_M = 6.0;

static std::vector<CDT::VertInd> calculate_boundary(const std::vector<Cone>& cones, const ConeColor c) {
    size_t start = 0;
    std::unordered_set<size_t> unvisited_cones { };
    for (size_t i = 0; i < cones.size(); ++i) {
        if (cones[i].c == c) {
            start = i;
            unvisited_cones.insert(i);
        }
    }
    size_t at = start;

    std::vector<CDT::VertInd> path;
    path.reserve(unvisited_cones.size());
    path.push_back(at);
    unvisited_cones.erase(at);
    while (not unvisited_cones.empty()) {
        // find cone with shortest distance to cones[at]
        size_t closest_cone = 0;
        double closest_dist = std::numeric_limits<double>::max();
        for (const size_t i : unvisited_cones) {
            if (const double dist = std::hypot(cones[at].x - cones[i].x, cones[at].y - cones[i].y); dist < closest_dist) {
                closest_dist = dist;
                closest_cone = i;
            }
        }
        if (closest_cone == start) {
            break;
        }
        unvisited_cones.erase(closest_cone);
        path.push_back(closest_cone);
        at = closest_cone;
    }

    return path;
}

static double mod(const double a, const double b) {
    const double result = std::fmod(a, b);
    return result >= 0 ? result : result + b;
}

// resample the closed polyline through `points` every CENTER_LINE_SPACING_M, then smooth it as a periodic smoothing
// spline: minimize sum |p_i - c_i|^2 + lambda sum |(p_{i-1} - 2 p_i + p_{i+1}) / ds^2|^2, i.e. solve (I + lambda D^T D) p = c
// with D the cyclic second difference. this is a low pass filter with gain 1 / (1 + lambda k^4) at wavenumber k, so
// lambda = (CENTER_LINE_SMOOTHING_M / 2 pi)^4 halves wiggles of wavelength CENTER_LINE_SMOOTHING_M and keeps longer ones
static std::vector<Eigen::Vector2d> smooth_closed_loop(const std::vector<Cone>& points) {
    const size_t m = points.size();
    std::vector<double> prefix(m + 1, 0.0);
    for (size_t i = 1; i <= m; ++i) {
        const Cone& a = points[i - 1];
        const Cone& b = points[i % m];
        prefix[i] = prefix[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
    }
    const double length = prefix[m];
    const auto n = static_cast<Eigen::Index>(std::max<double>(8, std::round(length / CENTER_LINE_SPACING_M)));
    const double ds = length / static_cast<double>(n);

    // uniform resampling, linearly along the polyline
    Eigen::MatrixX2d c(n, 2);
    size_t seg = 0;
    for (Eigen::Index i = 0; i < n; ++i) {
        const double s = static_cast<double>(i) * ds;
        while (seg + 1 < m and prefix[seg + 1] <= s) {
            ++seg;
        }
        const Cone& a = points[seg];
        const Cone& b = points[(seg + 1) % m];
        const double seg_len = prefix[seg + 1] - prefix[seg];
        const double f = seg_len > 0 ? (s - prefix[seg]) / seg_len : 0.0;
        c.row(i) << a.x + f * (b.x - a.x), a.y + f * (b.y - a.y);
    }

    constexpr double two_pi = 2 * 3.14159265358979323846;
    const double lambda = std::pow(CENTER_LINE_SMOOTHING_M / two_pi, 4) / std::pow(ds, 4);
    // D^T D for the cyclic second difference is the cyclic stencil (1, -4, 6, -4, 1)
    std::vector<Eigen::Triplet<double>> triplets;
    constexpr std::array<double, 5> stencil { 1, -4, 6, -4, 1 };
    for (Eigen::Index i = 0; i < n; ++i) {
        triplets.emplace_back(i, i, 1.0);
        for (Eigen::Index k = -2; k <= 2; ++k) {
            triplets.emplace_back(i, ((i + k) % n + n) % n, lambda * stencil[k + 2]);
        }
    }
    Eigen::SparseMatrix<double> system(n, n);
    system.setFromTriplets(triplets.begin(), triplets.end());
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(system);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("center line smoothing failed");
    }
    const Eigen::MatrixX2d p = solver.solve(c);

    std::vector<Eigen::Vector2d> out(static_cast<size_t>(n));
    for (Eigen::Index i = 0; i < n; ++i) {
        out[static_cast<size_t>(i)] = p.row(i).transpose();
    }
    return out;
}

void compute_path_from_percepted_cones() {
    ScopeTimer s { "compute_path timer" };
    static CDT::Triangulation<double> cdt_cache;
    static size_t seen_cones = 0;
    if (seen_cones == cones.size()) {
        return;
    }
    const auto start = std::next(cones.begin(), static_cast<std::vector<Cone>::difference_type>(seen_cones));
    cdt_cache.insertVertices(
        start, cones.end(), [](const Cone& c) { return c.x; }, [](const Cone& c) { return c.y; });

    // calculate boundary edges and insert them into the triangulation
    std::vector<CDT::Edge> edges { };
    const std::vector<CDT::VertInd> blue_boundary = calculate_boundary(cones, ConeColor::BLUE);
    for (size_t i = 0; i < blue_boundary.size(); ++i) {
        edges.emplace_back(blue_boundary[i], blue_boundary[(i + 1) % blue_boundary.size()]);
    }
    const std::vector<CDT::VertInd> yellow_boundary = calculate_boundary(cones, ConeColor::YELLOW);
    for (size_t i = 0; i < yellow_boundary.size(); ++i) {
        edges.emplace_back(yellow_boundary[i], yellow_boundary[(i + 1) % yellow_boundary.size()]);
    }
    // compute corresponding triangulation
    CDT::Triangulation<double> cdt = cdt_cache;
    cdt.insertEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();
    offline_boundary_edges = cdt.fixedEdges;
    offline_inner_edges = CDT::extractEdgesFromTriangles(cdt.triangles);
    const size_t original_num_edges = offline_inner_edges.size();
    for (const auto e : offline_boundary_edges) {
        offline_inner_edges.erase(e);
    }
    assert(offline_inner_edges.size() + offline_boundary_edges.size() == original_num_edges);

    // construct center line
    std::vector<Cone> _centers = { };
    for (const auto e : offline_inner_edges) {
        // get center point of e
        const Cone& c1 = cones[e.v1()];
        const Cone& c2 = cones[e.v2()];
        _centers.emplace_back((c1.x + c2.x) / 2, (c1.y + c2.y) / 2, ConeColor::CENTER);
    }
    const std::vector<CDT::VertInd> _center_idxs = calculate_boundary(_centers, ConeColor::CENTER);
    center_points.clear();
    center_points.reserve(_center_idxs.size());
    for (const size_t i : _center_idxs) {
        center_points.push_back(_centers[i]);
    }

    // the center points are midpoints of cone pairs, so they zig-zag by a few tens of cm. interpolating them exactly
    // turns that noise into ~1 m radius wiggles that the MPC tries (and fails) to follow, so fit a smooth loop instead
    const std::vector<Eigen::Vector2d> smoothed = smooth_closed_loop(center_points);

    // center line parameterization prefix (arc length along the smoothed loop)
    const size_t n = smoothed.size();
    alglib::real_1d_array center_line_len_prefix, xs, ys;
    center_line_len_prefix.setlength(static_cast<alglib::ae_int_t>(n + 1));
    xs.setlength(static_cast<alglib::ae_int_t>(n + 1));
    ys.setlength(static_cast<alglib::ae_int_t>(n + 1));
    center_line_len_prefix[0] = 0;
    for (size_t i = 0; i <= n; ++i) {
        // cycle around: the last knot repeats the first
        const Eigen::Vector2d& c = smoothed[i % n];
        xs[i] = c.x();
        ys[i] = c.y();
        if (i > 0) {
            center_line_len_prefix[i] = center_line_len_prefix[i - 1] + (c - smoothed[i - 1]).norm();
        }
    }
    center_line_length = center_line_len_prefix[n];

    // the center line is a closed loop, so use periodic boundary conditions (boundtype -1). the default (parabolically
    // terminated) ends leave the tangent and curvature discontinuous at the seam, which the MPC sees as a kink in the track
    const alglib::ae_int_t n_knots = static_cast<alglib::ae_int_t>(n + 1);
    alglib::spline1dbuildcubic(center_line_len_prefix, xs, n_knots, -1, 0.0, -1, 0.0, x_spline);
    alglib::spline1dbuildcubic(center_line_len_prefix, ys, n_knots, -1, 0.0, -1, 0.0, y_spline);
}

double project(const double x, const double y) {
    // const ScopeTimer s { "project timer" };
    static constexpr uint32_t samples = 50;
    static constexpr double eps = 0;

    double best_dist = std::numeric_limits<double>::max(), best_t = 0;
    const double step_size = center_line_length / samples - eps;
    assert(step_size > 0);
    for (double at_t = eps; at_t < center_line_length; at_t += step_size) { // NOLINT(*-flp30-c)
        const double t = project(x, y, at_t);
        const double d = std::hypot(
            alglib::spline1dcalc(x_spline, t) - x,
            alglib::spline1dcalc(y_spline, t) - y);
        if (d < best_dist) {
            best_dist = d;
            best_t = t;
        }
    }
    return best_t;
}
double project(const double x0, const double y0, double t) {
    // just good to double check
    assert(0 <= t);
    assert(t <= center_line_length);

    // newton stepping ????
    for (uint32_t i = 0; i < 20; i++) {
        double l_x, l_dx, l_ddx;
        alglib::spline1ddiff(x_spline, t, l_x, l_dx, l_ddx);
        double l_y, l_dy, l_ddy;
        alglib::spline1ddiff(y_spline, t, l_y, l_dy, l_ddy);
        // ||l'(t)||_2 + (l(t) - x) dot l''(t)
        const double f_prime = (l_dx * l_dx + l_dy * l_dy) + ((l_x - x0) * l_ddx + (l_y - y0) * l_ddy);
        // (l(t) - x) dot l'(t)
        const double f = (l_x - x0) * l_dx + (l_y - y0) * l_dy;
        const double step = f / f_prime;
        t = mod(t - step, center_line_length);
        if (step < 1e-6) {
            return t;
        }
    }
    std::cerr << "project did not converge at x=" << x0 << "and y=" << y0 << std::endl;
    throw std::runtime_error("project did not converge");
}

// apply the chain rule to lift f(t) onto a dual2nd t, given f, f' and f'' evaluated at val(t).
// a dual2nd is (a + b e1) + (c + d e1) e2, so f(t) = f(a) + f'(a) b e1 + f'(a) c e2 + (f'(a) d + f''(a) b c) e1 e2
static autodiff::dual2nd chain_rule(const autodiff::dual2nd& t, const double f, const double df, const double ddf) {
    autodiff::dual2nd out;
    out.val.val = f;
    out.val.grad = df * t.val.grad;
    out.grad.val = df * t.grad.val;
    out.grad.grad = df * t.grad.grad + ddf * t.val.grad * t.grad.val;
    return out;
}

Eigen::Vector2<autodiff::dual2nd> spline_t(const autodiff::dual2nd& t_dual) {
    // the center line is a closed loop, so wrap t onto a single lap instead of extrapolating past the ends of the spline
    const double t = mod(autodiff::val(t_dual), center_line_length);
    double l_x, l_dx, l_ddx;
    alglib::spline1ddiff(x_spline, t, l_x, l_dx, l_ddx);
    double l_y, l_dy, l_ddy;
    alglib::spline1ddiff(y_spline, t, l_y, l_dy, l_ddy);

    return { chain_rule(t_dual, l_x, l_dx, l_ddx), chain_rule(t_dual, l_y, l_dy, l_ddy) };
}

Eigen::Vector2<autodiff::dual2nd> spline_t_prime(const autodiff::dual2nd& t_dual) {
    const double t = mod(autodiff::val(t_dual), center_line_length);

    double l_x, l_dx, l_ddx;
    alglib::spline1ddiff(x_spline, t, l_x, l_dx, l_ddx);
    double l_y, l_dy, l_ddy;
    alglib::spline1ddiff(y_spline, t, l_y, l_dy, l_ddy);

    // alglib doesn't expose l'''(t) (piecewise constant for a cubic spline), so approximate it as 0
    return { chain_rule(t_dual, l_dx, l_ddx, 0), chain_rule(t_dual, l_dy, l_ddy, 0) };
}