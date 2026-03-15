#pragma once
#include "types.hpp"
#include <vector>

class Triangulation {
    struct Triangle {
        size_t a, b, c; // indices of the cones that form the triangle
        Triangle(size_t _a, size_t _b, size_t _c)
            : a(_a)
            , b(_b)
            , c(_c)
        {
        }
        bool contains(const Cone& cone, const ConeList& existing_cones) const
        {
            // check if cone is inside the triangle formed by cones a, b, c
            const Cone& cone_a = existing_cones[a];
            const Cone& cone_b = existing_cones[b];
            const Cone& cone_c = existing_cones[c];
            const double area_abc = 0.5 * std::abs(cone_a.x * (cone_b.y - cone_c.y) + cone_b.x * (cone_c.y - cone_a.y) + cone_c.x * (cone_a.y - cone_b.y));
            const double area_abp = 0.5 * std::abs(cone_a.x * (cone_b.y - cone.y) + cone_b.x * (cone.y - cone_a.y) + cone.x * (cone_a.y - cone_b.y));
            const double area_acp = 0.5 * std::abs(cone_a.x * (cone_c.y - cone.y) + cone_c.x * (cone.y - cone_a.y) + cone.x * (cone_a.y - cone_c.y));
            const double area_bcp = 0.5 * std::abs(cone_b.x * (cone_c.y - cone.y) + cone_c.x * (cone.y - cone_b.y) + cone.x * (cone_b.y - cone_c.y));
            return std::abs(area_abc - area_abp - area_acp - area_bcp) < 1e-6;
        }
    };

    class TrianglePyramid {
        std::vector<std::shared_ptr<TrianglePyramid>> children {};
        Triangle t;

    public:
        bool is_intact() { return children.empty(); }
        std::shared_ptr<TrianglePyramid>
        find_triangle(const Cone& cone, const ConeList& existing_cones)
        {
            for (const auto& child : children) {
                if (child->t.contains(cone, existing_cones)) {
                    if (child->is_intact()) {
                        return child;
                    }
                    return child->find_triangle(cone, existing_cones);
                }
            }
            throw std::runtime_error("Cone is not contained in any triangle");
        }
        TrianglePyramid() = delete;
        TrianglePyramid(const Triangle& _t)
            : t(_t)
        {
        }
        TrianglePyramid(size_t _a, size_t _b, size_t _c)
            : t { _a, _b, _c }
        {
        }
    };

    TrianglePyramid root { 0, 1, 2 }; // super triangle that contains all cones

    static std::unordered_map<size_t, std::vector<size_t>> adj {
        { 0, { 1, 2 } }, { 1, { 0, 2 } }, { 2, { 0, 1 } }
    }; // adj of the edges
public:
    void triangulate(const std::vector<Cone>& cones);
}