/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>

#include <vector>

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

/** One segment of a Reeds-Shepp path: a circular arc of radius
 * `turningRadius` turning left ('L') or right ('R'), or a straight line ('S').
 * `length` [m] is the traveled arc length, negative for reverse motion. */
struct ReedsSheppSegment
{
    char   type   = 'S';
    double length = 0;
};

/** The shortest Reeds-Shepp path from `from` to `to` as a sequence of
 * segments (zero-length segments omitted); its total length equals
 * reeds_shepp_distance(). Precondition: `turningRadius > 0`. */
std::vector<ReedsSheppSegment> reeds_shepp_path(
    const mrpt::math::TPose2D& from, const mrpt::math::TPose2D& to,
    double turningRadius);

/** A short Reeds-Shepp path from `from` to the point `to`, with a free final
 * heading, as a sequence of segments (empty if `to` is the position of
 * `from`). It is the reeds_shepp_path() to `to` with the best of a few final
 * headings: those of the four arc-then-straight paths, if `to` lies outside
 * both turning circles of `from` (the shortest path in most cases, and at most
 * 0.07 turning radii longer otherwise); else, a sampled and locally refined
 * heading (the shortest path, up to the refinement tolerance).
 * Precondition: `turningRadius > 0`. */
std::vector<ReedsSheppSegment> reeds_shepp_path_to_point(
    const mrpt::math::TPose2D& from, const mrpt::math::TPoint2D& to,
    double turningRadius);

/** Applies a sequence of segments to `from` (exact circular-arc geometry). */
mrpt::math::TPose2D reeds_shepp_apply(
    const mrpt::math::TPose2D&            from,
    const std::vector<ReedsSheppSegment>& segments, double turningRadius);

}  // namespace mpp
