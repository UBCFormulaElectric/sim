#pragma once
#include "types.hpp"

void sim_step(VehicleState& state, const ControlOutput& control, const int dt);