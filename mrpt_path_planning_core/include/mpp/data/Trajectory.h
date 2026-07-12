/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>

#include <vector>

namespace mpp
{
/** One point of a reference path for the TrajectoryFollower: a pose plus a
 * desired speed cap at that point. */
struct RefPoint
{
    RefPoint() = default;
    RefPoint(const mrpt::math::TPose2D& p, double v) : pose(p), target_speed(v)
    {
    }

    mrpt::math::TPose2D pose;

    /** [m/s] Desired speed cap along the arc-length at this point. A value <= 0
     * means "unspecified": the follower uses its configured max speed. */
    double target_speed = 0;
};

/** A planner-agnostic reference path: a pose+speed polyline, interpolated by
 * arc-length. It is *not* required to be kinematically feasible: the follower's
 * pure-pursuit law smooths over it. Any source (a planner, a recording, a
 * hand-drawn path) can produce one. */
using Trajectory = std::vector<RefPoint>;

}  // namespace mpp
