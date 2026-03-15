#pragma once
#include "types.hpp"
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

// inspired by https://youtu.be/1TUUevxkvp4

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

        std::shared_ptr<TrianglePyramid> find_triangle(const Cone& cone, const ConeList& existing_cones)
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

    class ConeList {
        const std::optional<std::span<const Cone>> cones;

    public:
        ConeList()
            : cones(std::nullopt)
        {
        }

        void set_cones(std::span<const Cone> _cones)
        {
            cones.emplace(_cones);
        }

        const Cone& operator[](const size_t cone_id) const
        {
            switch (cone_id) {
            case 0:
                return Cone { -100, -100, ConeColor::BLUE };
            case 1:
                return Cone { 100, -100, ConeColor::YELLOW };
            case 2:
                return Cone { 0, 100, ConeColor::BLUE };
            default:
                return cones.value()[cone_id - 3];
            }
        }
    };

    TrianglePyramid root { 0, 1, 2 }; // super triangle that contains all cones
    ConeList existing_cones {};

    /**
     * helpers
     */
    bool on_edge(const Cone& cone, const size_t a, const size_t b);
    bool edge_detect(const Cone& cone, size_t& a, size_t& b, size_t& c);
    void insert(const Cone& cone, const size_t cone_id);
    void legalize_edge(const size_t new_cone, const size_t cone_a, const size_t cone_b);

public:
    Triangulation() = default;
    static std::unordered_map<size_t, std::vector<size_t>> adj {
        { 0, { 1, 2 } }, { 1, { 0, 2 } }, { 2, { 0, 1 } }
    }; // adj of the edges
    void triangulate(const std::vector<Cone>& cones);
}