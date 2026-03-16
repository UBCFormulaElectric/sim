#include "controller.hpp"

Triangulation t {};

Triangulation& get_triangulation()
{
    return t;
}

ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones)
{
    // Implementation for compute function
    t.update(cones);
    // t.adj will contain the adj list
    return { 0, 0 };
}