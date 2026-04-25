#pragma once
#include "types.hpp"

ControlOutput compute(const VehicleState& ve);

enum class DriverlessState {
    IDLE,
    ONLINE_MAPPING, // use for autocross, acceleration, skidpad, etc.
    TRACKDRIVE_MAPPING,
    TRACKDRIVE_RUNNING
};