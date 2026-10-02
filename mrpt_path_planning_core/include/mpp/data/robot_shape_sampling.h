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

/** Returns the robot footprint as a polygon: the polygon itself, a regular
 * `circleSegments`-gon inscribing the circle for a radius, or an empty polygon
 * for a `std::monostate` shape. Useful to publish or compare footprints. */
mrpt::math::TPolygon2D robotShapeAsPolygon(
    const RobotShape& shape, std::size_t circleSegments = 16);

/** Whether two footprints are the same, within `tolerance` [m]: every vertex
 * of each polygon (see robotShapeAsPolygon()) must be within `tolerance` of
 * the other polygon boundary. */
bool sameRobotShape(
    const mrpt::math::TPolygon2D& a, const mrpt::math::TPolygon2D& b,
    double tolerance = 0.01);

}  // namespace mpp
