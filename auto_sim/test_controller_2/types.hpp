#pragma once

struct ControlOutput {
    // in local frame
    double ax;
    // no frame
    double omega_dot;
    ControlOutput(const double _ax, const double _omega_dot)
        : ax(_ax), omega_dot(_omega_dot) { }
};

struct VehicleState {
    // in global frame
    double x;
    double y;
    double theta;
    // in local frame
    double v_x;
    double v_y;
    // no frame
    double omega;
    VehicleState()
        : x(0), y(0), theta(0), v_x(0), v_y(0), omega(0) { }
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