#pragma once

#include "CDT.h"
#include "interpolation.h"
#include "types.hpp"

#include <vector>

/**
 * This computes the path from the given ALL cones offline.
 */
void compute_path_from_percepted_cones();
/**
 * Note this overload is MUCH faster than without at_t
 * @param x x position
 * @param y y position
 * @param at_t please use this to seed the optimization algorithm
 * @return t s.t. the point on the center line that is closest to (x, y) in terms of Euclidean distance
 */
double project(double x, double y, double at_t = 0);
/**
 * this variant tries a bunch of values of at_t, between [0, center_line_length], and returns the best one.
 * this is more expensive but can be more robust if the center line is very curvy and the optimization algorithm gets stuck in a local minimum.
 * @param x x position
 * @param y y position
 * @return
 */
double project(double x, double y);

// for debugging purposes only
inline CDT::EdgeUSet offline_boundary_edges;
inline CDT::EdgeUSet offline_inner_edges;
inline std::vector<Cone> center_points;
inline std::vector<CDT::VertInd> center_line_idxs;
inline alglib::spline1dinterpolant x_spline, y_spline;
inline double center_line_length;