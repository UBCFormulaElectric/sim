#include "path.hpp"
#include "perception.hpp"
#include "scopetimer.hpp"
#include <unordered_set>

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

    // center line parameterization prefix
    alglib::real_1d_array center_line_len_prefix;
    center_line_len_prefix.setlength(center_points.size() + 1);
    center_line_len_prefix[0] = 0;
    for (size_t i = 1; i <= center_points.size(); ++i) {
        const Cone& c1 = center_points[i - 1];
        const Cone& c2 = center_points[i % center_points.size()];
        center_line_len_prefix[i] = center_line_len_prefix[i - 1] + std::hypot(c1.x - c2.x, c1.y - c2.y);
    }
    center_line_length = center_line_len_prefix[center_points.size()];

    // draw a spline between all the center points
    alglib::real_1d_array xs, ys;
    xs.setlength(center_points.size() + 1);
    ys.setlength(center_points.size() + 1);
    for (size_t i = 0; i < center_points.size(); ++i) {
        const Cone& c = center_points[i];
        xs[i] = c.x;
        ys[i] = c.y;
    }
    // cycle around
    xs[center_points.size()] = center_points[0].x;
    ys[center_points.size()] = center_points[0].y;

    alglib::spline1dbuildcubic(center_line_len_prefix, xs, x_spline);
    alglib::spline1dbuildcubic(center_line_len_prefix, ys, y_spline);
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
    std::cout << "project did not converge at x=" << x0 << "and y=" << y0 << std::endl;
    throw std::exception("project did not converge");
}

Location spline_t(const double t) {
    return { alglib::spline1dcalc(x_spline, t), alglib::spline1dcalc(y_spline, t) };
}
