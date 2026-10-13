#pragma once

struct ControlOutput {
    // in local frame
    double ax;
    // no frame
    double omega_dot;
    // in local frame
    double delta;

    ControlOutput(const double _ax, const double _omega_dot, const double _delta)
        : ax(_ax), omega_dot(_omega_dot), delta(_delta) { }
};

struct VehicleState {
    // in global frame
    double x;
    double y;
    double theta;
    // in local frame
    double v_x;
    double v_y;
    // no frame, relative to world
    double omega;
    VehicleState()
        : x(0), y(0), theta(0), v_x(0), v_y(0), omega(0) { }
    VehicleState(const double _x, const double _y, const double _theta, const double _v_x, const double _v_y, const double _omega)
        : x(_x), y(_y), theta(_theta), v_x(_v_x), v_y(_v_y), omega(_omega) { }
};

enum class ConeColor {
    BLUE,
    YELLOW,
    ORANGE,
    CENTER
};
struct Cone {
    double x;
    double y;
    ConeColor c;
    Cone(const double _x, const double _y, const ConeColor _c)
        : x(_x), y(_y), c(_c) { }
};