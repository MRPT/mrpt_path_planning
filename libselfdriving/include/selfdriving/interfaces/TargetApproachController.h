/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <selfdriving/data/VehicleLocalizationState.h>
#include <selfdriving/data/VehicleOdometryState.h>
#include <selfdriving/data/Waypoints.h>

namespace selfdriving
{
using mrpt::kinematics::CVehicleVelCmd;

struct TargetApproachInput
{
    VehicleLocalizationState vls;
    VehicleOdometryState     vos;
    Waypoint                 target;
    std::optional<Waypoint>  previous;
};

struct TargetApproachOutput
{
    TargetApproachOutput() = default;

    /** Should be set to true if the controller actually did its job, false to
     * let the parent NavEngine in control of the robot motion (and then ignore
     * the rest of fields in this struct). */
    bool handled = false;

    // bool freePathFound   = false;
    // bool targetOvershoot = false;

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

}  // namespace selfdriving
