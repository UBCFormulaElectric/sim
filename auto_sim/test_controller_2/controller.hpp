#pragma once
#include "CDT.h"
#include "types.hpp"
#include <vector>

const CDT::EdgeUSet& get_triangulation();
ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones);