#pragma once
#include "CDT.h"
#include "types.hpp"
#include <vector>

ControlOutput compute(const VehicleState& ve);

enum class DriverlessState {
    IDLE,
    ONLINE_MAPPING, // use for autocross, acceleration, skidpad, etc.
    TRACKDRIVE_MAPPING,
    TRACKDRIVE_RUNNING
};

// NOTE everything under this line should be private, but we are exposing it for testing purposes
/**
 * @param cones cones to calculate the boundary from
 * @param c the color of the cones to calculate the boundary for
 * @return a vector of vertex indices that form the boundary of the given color. The order of the vertices is not guaranteed to be clockwise or counterclockwise.
 */
std::vector<CDT::VertInd> calculate_boundary(const std::vector<Cone>& cones, ConeColor c);
/**
 * This computes the path from the given ALL cones offline.
 * @param cones
 */
void compute_path(const std::vector<Cone>& cones);
/**
 * @return offline edges
 */
const CDT::EdgeUSet& get_offline_edges();
/**
 * @return boundary edges
 */
const CDT::EdgeUSet& get_boundary_edges();
/**
 * @return center points
 */
const std::vector<Cone>& get_center_points();
/**
 * @return center line vertices
 */
const std::vector<CDT::VertInd>& get_center_line();

// online path planning
void update_cone_positions(const std::vector<Cone>& cones);
const CDT::EdgeUSet& get_triangulation();