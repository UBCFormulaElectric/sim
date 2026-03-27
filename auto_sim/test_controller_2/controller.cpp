#include "controller.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>

static CDT::Triangulation<double> cdt;
static CDT::EdgeUSet edges;

const CDT::EdgeUSet& get_triangulation() {
    return edges;
}

ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones) {
    // Implementation for compute function
    static size_t seen_points = 0;
    if (seen_points < cones.size()) {
        const auto start_time = std::chrono::high_resolution_clock::now();
        const auto start = std::next(
            cones.begin(),
            static_cast<std::vector<Cone>::difference_type>(std::min(seen_points, cones.size())));
        cdt.insertVertices(start, cones.end(), [](const Cone& c) { return c.x; }, [](const Cone& c) { return c.y; });
        // CDT::Triangulation<double> cpy = cdt;
        // cpy.eraseOuterTriangles();
        // std::cout << "number of vertices in cpy " << cpy.vertices.size() << std::endl;
        cdt.eraseSuperTriangle();
        edges = CDT::extractEdgesFromTriangles(cdt.triangles);
        std::cout << "number of edges " << edges.size() << std::endl;
        seen_points = cones.size();
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Time taken to insert vertices and extract edges: " << duration << " us" << std::endl;
    }
    // cdt.eraseSuperTriangle();
    // t.adj will contain the adj list
    return { 0, 0 };
}