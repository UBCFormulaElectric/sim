#pragma once
#include "triangulation.hpp"
#include "types.hpp"
#include <vector>

Triangulation& get_triangulation();
ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones);