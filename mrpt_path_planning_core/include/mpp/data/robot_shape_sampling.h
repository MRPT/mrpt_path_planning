/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/TrajectoriesAndRobotShape.h>  // RobotShape
#include <mrpt/math/TPoint2D.h>

#include <cstddef>
#include <vector>

namespace mpp
{
/** Builds a set of robot-frame points sampling the robot footprint boundary.
 *
 * For a polygon: its vertices plus points subdividing each edge no coarser than
 * `resolution`. For a radius: a ring of points at that radius. For a
 * `std::monostate` shape: an empty vector (caller decides the fallback, e.g.
 * sampling only the reference point).
 *
 * The total number of samples is capped at `maxSamples`: sampling finer than
 * `resolution` is wasted, and for very large footprints the perimeter step is
 * coarsened to keep the count bounded (this may run on hot paths).
 */
std::vector<mrpt::math::TPoint2D> footprintSamplePoints(
    const RobotShape& shape, double resolution, std::size_t maxSamples = 128);

}  // namespace mpp
