/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/datetime.h>

#include <string>

namespace mpp
{
/** Data returned by VehicleMotionInterface::get_odometry() */
struct VehicleOdometryState
{
    /** Set to true if data could be retrieved from the robot system
     * successfully, set to false upon any critical error that made it
     * impossible to fill in the data fields in this structure. */
    bool valid = false;

    /** The latest robot raw odometry pose.
     * (x,y: meters, phi: radians).
     * Axes: +x=forward, +y=vehicle left, +phi=CCW rotation.  */
    mrpt::math::TPose2D odometry;

    /** The latest robot velocity, in its own body local frame of reference.
     * (vx,vy: m/s, omega: rad/s)
     * Axes: +vx=forward, +vy=vehicle left, +omega=CCW rotation.  */
    mrpt::math::TTwist2D odometryVelocityLocal;

    /** The timestamp for the read pose and velocity values. Use
     * mrpt::system::now() unless you have something more accurate. */
    mrpt::system::TTimeStamp timestamp;

    /** There exists an action waiting for execution after the current
     * under-execution one.
     * See: VehicleMotionInterface::motion_execute() for a discussion.
     */
    bool pendedActionExists = false;
};

}  // namespace mpp
