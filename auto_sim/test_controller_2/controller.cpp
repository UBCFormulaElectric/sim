#include "controller.hpp"
#include "triangulation.hpp"

ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones)
{
    // Implementation for compute function
    triangulate(cones);
    return { 0, 0 };
}