/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/datetime.h>

#include <string>
#include <vector>

namespace mpp
{
/** One sample of the short rolling reference the follower hands to the vehicle
 * interface to execute. */
struct TrajSample
{
    /** [s] Time from the parent SampledTrajectory `stamp` (t=0 at the sample
     * the robot should be executing "now"). */
    double t = 0;

    /** Desired pose, expressed in the parent's `frame_id`. */
    mrpt::math::TPose2D pose;

    /** Desired body-frame velocity (vx, vy, omega). Frame-invariant. */
    mrpt::math::TTwist2D twist;

    /** Safety speed factor applied to this sample (1 = full profile speed, 0 =
     * commanded stop). For debugging / visualization. */
    double speed_scale = 1.0;
};

/** A short-horizon, safety-scaled predicted trajectory: the exact motion the
 * follower wants the robot to make over the next horizon. It is simultaneously
 * the command to execute, the rollout the safety check sweeps, and a
 * self-expiring latency buffer (each sample is time-stamped). */
struct SampledTrajectory
{
    /** Frame the sample poses are expressed in (e.g. "odom"). */
    std::string frame_id;

    /** Wall time of t=0. */
    mrpt::system::TTimeStamp stamp = INVALID_TIMESTAMP;

    /** Samples, in strictly increasing `t`. */
    std::vector<TrajSample> points;

    bool empty() const { return points.empty(); }
};

}  // namespace mpp
