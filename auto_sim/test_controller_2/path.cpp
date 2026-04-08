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

static float mod(const float a, const float b) {
    return a - b * std::floor(a / b);
}
static double mod(const double a, const double b) {
    return a - b * std::floor(a / b);
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
    center_points = { };
    for (const auto e : offline_inner_edges) {
        // get center point of e
        const Cone& c1 = cones[e.v1()];
        const Cone& c2 = cones[e.v2()];
        center_points.emplace_back((c1.x + c2.x) / 2, (c1.y + c2.y) / 2, ConeColor::CENTER);
    }
    center_line_idxs = calculate_boundary(center_points, ConeColor::CENTER);

    // center line parameterization prefix
    alglib::real_1d_array center_line_len_prefix;
    center_line_len_prefix.setlength(center_line_idxs.size() + 1);
    for (size_t i = 1; i <= center_line_idxs.size(); ++i) {
        const Cone& c1 = center_points[center_line_idxs[i - 1]];
        const Cone& c2 = center_points[center_line_idxs[i]];
        center_line_len_prefix[i] = center_line_len_prefix[i - 1] + std::hypot(c1.x - c2.x, c1.y - c2.y);
    }
    center_line_length = center_line_len_prefix[center_line_idxs.size()];

    // draw a spline between all the center points
    alglib::real_1d_array xs, ys { };
    xs.setlength(center_line_idxs.size() + 1);
    ys.setlength(center_line_idxs.size() + 1);
    for (size_t i = 0; i < center_line_idxs.size(); ++i) {
        const Cone& c = center_points[center_line_idxs[i]];
        xs[i] = c.x;
        ys[i] = c.y;
    }
    // cycle around
    xs[center_line_idxs.size()] = center_points[center_line_idxs[0]].x;
    ys[center_line_idxs.size()] = center_points[center_line_idxs[0]].y;

    alglib::spline1dbuildcubic(xs, center_line_len_prefix, x_spline);
    alglib::spline1dbuildcubic(ys, center_line_len_prefix, y_spline);
}

double project(const double x, const double y) {
    static constexpr uint32_t samples = 10;
    double best_dist = std::numeric_limits<double>::max(), best_t = 0;
    for (double at_t = 0; at_t <= center_line_length; at_t += center_line_length / samples) { // NOLINT(*-flp30-c)
        if (const double dist = std::hypot(
                alglib::spline1dcalc(x_spline, at_t) - x,
                alglib::spline1dcalc(y_spline, at_t) - y);
            dist < best_dist) {
            best_t = project(x, y, at_t);
            best_dist = std::hypot(
                alglib::spline1dcalc(x_spline, best_t) - x,
                alglib::spline1dcalc(y_spline, best_t) - y);
        }
    }
    return best_t;
}
double project(const double x, const double y, double at_t) {
    // just good to double check
    assert(0 <= at_t);
    assert(at_t <= center_line_length);

    double dd;
    double at_x, at_dx;
    alglib::spline1ddiff(x_spline, at_t, at_x, at_dx, dd);
    double at_y, at_dy;
    alglib::spline1ddiff(y_spline, at_t, at_y, at_dy, dd);

    // newton stepping ????
    for (uint32_t i = 0; i < 20; i++) {
        const double f_prime = 2 * (at_x * at_dy + at_y * at_dx) - (x * at_dy + y * at_dx);
        const double f = (at_x * at_dy + at_y * at_dx) - (x * at_dy + y * at_dx);
        const double step = f / f_prime;
        at_t -= step;
        if (step < 1e-6) {
            return at_t;
        }
    }
    throw std::runtime_error("project did not converge");
}
