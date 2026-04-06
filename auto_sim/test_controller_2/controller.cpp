#include "controller.hpp"
#include "scopetimer.hpp"

std::vector<CDT::VertInd> calculate_boundary(const std::vector<Cone>& cones, const ConeColor c) {
    size_t start = 0;
    std::unordered_set<size_t> unvisited_cones {};
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

static double get_cone_x(const Cone& c) {
    return c.x;
}
static double get_cone_y(const Cone& c) {
    return c.y;
}

static CDT::EdgeUSet offline_inner_edges;
static CDT::EdgeUSet offline_boundary_edges;
static std::vector<Cone> center_points;
static std::vector<CDT::VertInd> center_line;
void compute_path(const std::vector<Cone>& cones) {
    ScopeTimer s { "offline triangulation timer" };
    CDT::Triangulation<double> cdt;
    cdt.insertVertices(cones.begin(), cones.end(), get_cone_x, get_cone_y);

    // calculate boundary edges and insert them into the triangulation
    std::vector<CDT::Edge> edges {};
    const std::vector<CDT::VertInd> blue_boundary = calculate_boundary(cones, ConeColor::BLUE);
    for (size_t i = 0; i < blue_boundary.size(); ++i) {
        edges.emplace_back(blue_boundary[i], blue_boundary[(i + 1) % blue_boundary.size()]);
    }
    const std::vector<CDT::VertInd> yellow_boundary = calculate_boundary(cones, ConeColor::YELLOW);
    for (size_t i = 0; i < yellow_boundary.size(); ++i) {
        edges.emplace_back(yellow_boundary[i], yellow_boundary[(i + 1) % yellow_boundary.size()]);
    }
    cdt.insertEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();
    offline_boundary_edges = cdt.fixedEdges;
    offline_inner_edges = CDT::extractEdgesFromTriangles(cdt.triangles);
    const size_t original_num_edges = offline_inner_edges.size();
    for (const auto e : offline_boundary_edges) {
        offline_inner_edges.erase(e);
    }
    assert(offline_inner_edges.size() + offline_boundary_edges.size() == original_num_edges);

    center_points = {};
    for (const auto e : offline_inner_edges) {
        // get center point of e
        const Cone& c1 = cones[e.v1()];
        const Cone& c2 = cones[e.v2()];
        center_points.emplace_back((c1.x + c2.x) / 2, (c1.y + c2.y) / 2, ConeColor::CENTER);
    }
    center_line = calculate_boundary(center_points, ConeColor::CENTER);
}

const CDT::EdgeUSet& get_offline_edges() {
    return offline_inner_edges;
}
const CDT::EdgeUSet& get_boundary_edges() {
    return offline_boundary_edges;
}
const std::vector<Cone>& get_center_points() {
    return center_points;
}
const std::vector<CDT::VertInd>& get_center_line() {
    return center_line;
}

static CDT::EdgeUSet edges;
void update_cone_positions(const std::vector<Cone>& cones) {
    static CDT::Triangulation<double> cdt;
    static size_t seen_cones = 0;
    if (seen_cones == cones.size()) {
        return;
    }

    ScopeTimer s { "online triangulation timer" };
    const auto start = std::next(cones.begin(), static_cast<std::vector<Cone>::difference_type>(seen_cones));
    cdt.insertVertices(
        start, cones.end(), [](const Cone& c) { return c.x; }, [](const Cone& c) { return c.y; });
    CDT::Triangulation<double> cpy = cdt;
    cpy.eraseSuperTriangle();
    edges = CDT::extractEdgesFromTriangles(cpy.triangles);
}

const CDT::EdgeUSet& get_triangulation() {
    return edges;
}

ControlOutput compute(const VehicleState& ve) {
    // t.adj will contain the adj list
    return { 0, 0 };
}