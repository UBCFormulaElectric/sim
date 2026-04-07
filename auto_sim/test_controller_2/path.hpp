#pragma once

#include "CDT.h"
#include "types.hpp"
#include <vector>

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
 * @param cones
 */
void update_cone_positions(const std::vector<Cone>& cones);

// NOTE everything under this line should be private, but we are exposing it for testing purposes
const CDT::EdgeUSet& get_offline_edges();
const CDT::EdgeUSet& get_boundary_edges();
const std::vector<Cone>& get_center_points();
const std::vector<CDT::VertInd>& get_center_line();
const CDT::EdgeUSet& get_triangulation();

double project(double x, double y, const Cone& c0, const Cone& c1);