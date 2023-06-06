/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/data/VehicleLocalizationState.h>
#include <mpp/data/VehicleOdometryState.h>
#include <mpp/data/Waypoints.h>
#include <mpp/data/const_ref_t.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mpp/interfaces/VehicleMotionInterface.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mpp
{
using mrpt::kinematics::CVehicleVelCmd;

struct TargetApproachInput
{
    const_ref_t<VehicleLocalizationState>           vls;
    const_ref_t<VehicleOdometryState>               vos;
    mrpt::kinematics::CVehicleVelCmd::TVelCmdParams speedLimits;
    Waypoint                                        target;
    std::optional<Waypoint>                         previous;
    const_ref_t<TrajectoriesAndRobotShape>          ptgsAndShape;
    ObstacleSource::Ptr                             localSensedObstacleSource;
    VehicleMotionInterface::Ptr                     vehicleMotionInterface;
};

struct TargetApproachOutput
{
    TargetApproachOutput() = default;

    /** Should be set to true if the controller actually did its job, false to
     * let the parent NavEngine in control of the robot motion (and then ignore
     * the rest of fields in this struct). */
    bool handled = false;

    bool reachedDetected = false;

    /** If `handled==true`, this should be a motion primitive to execute on the
     * robot, or an empty pointer if we are still waiting for the completion of
     * a former motion command.
     */
    CVehicleVelCmd::Ptr generatedMotion;
};

/** Virtual interface for custom policies to take a robot towards a nearby
 * goal point with a final speed of zero (stopped). This is done without path
 * planning to avoid obstacles, just moving straight to the goal.
 *
 * Called by NavEngine, if provided by the user in the configuration file.
 */
class TargetApproachController : public mrpt::system::COutputLogger,
                                 public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(TargetApproachController)

   public:
    TargetApproachController()
        : mrpt::system::COutputLogger("TargetApproachController")
    {
    }
    virtual ~TargetApproachController();

    virtual TargetApproachOutput execute(const TargetApproachInput& in) = 0;

    /** Will be called in navigation steps where this interface is not required.
     *  Use it to reset any flag about pending or past maneuvering.
     */
    virtual void reset_state() = 0;
};

}  // namespace mpp
