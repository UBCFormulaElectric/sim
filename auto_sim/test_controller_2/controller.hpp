#pragma once
#include "types.hpp"
#include <vector>

ControlOutput compute(const VehicleState& ve, const std::vector<Cone>& cones);