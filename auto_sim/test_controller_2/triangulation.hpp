#pragma once
#include "types.hpp"
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// inspired by https://youtu.be/1TUUevxkvp4

class Triangulation {
    class ConeList {
        std::optional<std::span<const Cone>> cones;

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
            static const Cone cone0{ -100, -100, ConeColor::BLUE },
                cone1 = { 100, -100, ConeColor::YELLOW },
                cone2 = { 0, 100, ConeColor::BLUE };
            switch (cone_id) {
            case 0:
                return cone0;
            case 1:
                return cone1;
            case 2:
                return cone2;
            default:
                return cones.value()[cone_id - 3];
            }
        }
    };

    struct Triangle {
        size_t a, b, c; // indices of the cones that form the triangle
        Triangle(const size_t _a, const size_t _b, const size_t _c)
            : a(_a)
            , b(_b)
            , c(_c)
        {
        }
        [[nodiscard]] bool contains(const Cone& cone) const
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
        bool is_intact() const { return children.empty(); }

        std::shared_ptr<TrianglePyramid> find_triangle(const Cone& cone)
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
        explicit TrianglePyramid(const Triangle& _t)
            : t(_t)
        {
        }
        TrianglePyramid(const size_t _a, const size_t _b, const size_t _c)
            : t { _a, _b, _c }
        {
        }
    };

    TrianglePyramid root { 0, 1, 2 }; // super triangle that contains all cones
    ConeList existing_cones {};

    /**
     * helpers
     */
    [[nodiscard]] bool on_edge(const Cone& cone, size_t a, size_t b) const;
    bool edge_detect(const Cone& cone, size_t& a, size_t& b, size_t& c) const;
    void insert(const Cone& cone, size_t cone_id);
    void legalize_edge(size_t new_cone, size_t cone_a, size_t cone_b);

public:
    Triangulation() = default;
    std::unordered_map<size_t, std::vector<size_t>> adj {
        { 0, { 1, 2 } }, { 1, { 0, 2 } }, { 2, { 0, 1 } }
    }; // adj of the edges
    void update(const std::vector<Cone>& cones);
};
