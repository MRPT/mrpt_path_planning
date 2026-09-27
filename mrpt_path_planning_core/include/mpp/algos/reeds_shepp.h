/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>

namespace mpp
{
/** Length [m] of the shortest Reeds-Shepp path (forward and reverse motion,
 * circular arcs of radius `turningRadius` and straight segments, no obstacles)
 * from `from` to `to`. Precondition: `turningRadius > 0`.
 *
 * Any path of a vehicle whose curvature is bounded by 1/turningRadius is at
 * least this long, so this is a lower bound of the path length usable as an
 * admissible (and consistent) heuristic for full-pose goals.
 *
 * Based on the formulas of Reeds and Shepp (1990) as implemented in OMPL's
 * ReedsSheppStateSpace (BSD license, Rice University).
 */
double reeds_shepp_distance(
    const mrpt::math::TPose2D& from, const mrpt::math::TPose2D& to,
    double turningRadius);

}  // namespace mpp
