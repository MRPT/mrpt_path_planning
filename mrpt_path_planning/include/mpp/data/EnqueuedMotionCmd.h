/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/TPose2D.h>

namespace mpp
{
/** An odometry position condition used in EnqueuedMotionCmd */
struct EnqueuedCondition
{
    /** The center of the (x,y,phi) "pose volume" that will trigger the next
     * motion command, in *odometry* frame of reference. */
    mrpt::math::TPose2D position;

    /** The three lengths of the "pose volume" described above.
     * Only possitive numbers in all (x,y,phi) fields.
     *
     * Example:
     * - position  = [-2.0, 3.0, 45.0_deg]
     * - tolerance = [0.10, 0.10, 2.0_deg]
     * means the next command will be triggered when the robot odometry enters
     * the volume:
     *  - x   ∈ [-2.05, -1.95]
     *  - y   ∈ [ 2.95,  3.05]
     *  - phi ∈ [ 44_deg, 46_deg]
     *
     */
    mrpt::math::TPose2D tolerance;

    /** In seconds. This should hold a larger time than the ETA to the given
     * position. */
    double timeout = 10.0;
};

/** A "pending" motion command to be used in
 * VehicleMotionInterface::motion_execute() */
struct EnqueuedMotionCmd
{
    mrpt::kinematics::CVehicleVelCmd::Ptr nextCmd;
    EnqueuedCondition                     nextCondition;
};

}  // namespace mpp
