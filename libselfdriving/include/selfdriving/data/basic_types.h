/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <cstdlib>  // size_t

namespace selfdriving
{
/** Distances measured by PoseDistanceMetric, or "pseudodistances" of PTGs, that
 * is, distances along SE(2), including a weighted distance for rotations */
using distance_t = double;

/** TPS normalized distances in range [0,1] */
using normalized_distance_t = double;

/** Cost of a given trajectory or trajectory segment */
using cost_t = double;

/** Index of a trajectory in a PTG */
using trajectory_index_t = int;

using ptg_index_t = size_t;

/** Normalized speed in range [0,1] */
using normalized_speed_t = double;

/** Time duration in seconds */
using duration_seconds_t = double;

}  // namespace selfdriving
