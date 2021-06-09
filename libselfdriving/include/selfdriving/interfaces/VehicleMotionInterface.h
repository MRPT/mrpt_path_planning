/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/system/COutputLogger.h>
#include <selfdriving/data/EnqueuedMotionCmd.h>
#include <selfdriving/data/VehicleLocalizationState.h>
#include <selfdriving/data/VehicleOdometryState.h>

namespace selfdriving
{
using mrpt::kinematics::CVehicleVelCmd;

enum class STOP_TYPE : uint8_t
{
    REGULAR = 0,
    EMERGENCY
};

/** The virtual interface between a path follower / plan executor and a real
 * mobile platform (vehicle, robot), regarding controlling its motion.
 *
 * To enable the use of this library with a new robot, the user must define
 * a new class derived from this virtual class and implement all pure virtual
 * and the desired virtual methods according to the docs below.
 *
 * This class does not make assumptions about the kinematic model of the robot,
 * so it can work with either Ackermann, differential-driven or
 * holonomic robots. It will depend on the used PTGs, so checkout each PTG
 * documentation for the lenght and meaning of velocity commands.
 *
 * Assumptions: The platform...
 * - has a global localization system (e.g. particle filter) with
 *   respect to some fixed global frame of reference `map`, see
 *   get_localization().
 * - has odometry with a fixed global frame of reference `odom`, see
 *   get_odometry().
 * - is able to execute motion primitives according to one or more PTGs,
 * - is able to keep one immediately-running motion primitive and optionally
 *   another one in a queue, which will be executed when a given condition
 *   holds; see motion_execute(). This mechanism ensures accurately path
 *   following without computer-robot communication delays affecting the plan.
 *
 *
 */
class VehicleMotionInterface : public mrpt::system::COutputLogger
{
   public:
    using Ptr = std::shared_ptr<VehicleMotionInterface>;

    VehicleMotionInterface()
        : mrpt::system::COutputLogger("VehicleMotionInterface")
    {
    }
    virtual ~VehicleMotionInterface() = default;

    /** Provides access to the vehicle localization data.
     *
     * The implementation should not take too much time to return, so if it
     * might take more than ~10ms to ask the robot for the instantaneous data,
     * it may be good enough to return the latest cached values, updated in a
     * parallel thread.
     *
     * In case of a hardware/communication error, leave `valid=false` in the
     * return structure.
     */
    virtual VehicleLocalizationState get_localization() = 0;

    /** Provides access to the vehicle odometry data.
     *
     *
     * The implementation should not take too much time to return, so if it
     * might take more than ~10ms to ask the robot for the instantaneous data,
     * it may be good enough to return the latest cached values, updated in a
     * parallel thread.
     *
     * In case of a hardware/communication error, leave `valid=false` in the
     * return structure.
     */
    virtual VehicleOdometryState get_odometry() = 0;

    /** Sends a motion command to the vehicle "immediate" and "next" slots.
     *
     * A vehicle may accept one or more different implementation-specific
     * CVehicleVelCmd classes.
     *
     * This method resets the watchdog timer started with startWatchdog()
     *
     * The vehicle should be able to execute a motion primitive ("immediate"
     * slot) and holding a pending one ("next" slot) which will be moved to the
     * immediate slot when a given condition holds, leaving the "next" slot
     * free.
     *
     * Possible combinations of calls to this method:
     *
     *  1) `immediate` set, `next` not set: replaces whatever the vehicle was
     * doing and executes the given immediate command, dropping possible former
     * immediate and next commands.
     *
     *  2) `immediate` set, `next` set: replaces both vehicle execution "slots"
     * with the given commands.
     *
     *  3) `immediate` not set, `next` set: should be received by a vehicle only
     * while there is an "immediate" motion command under execution, and the
     * "next" slot is free. This stores a new "next" motion command, leaving the
     * currently-executing one untouched. When the "next condition" holds true,
     * the "next" slot will replace the "immediate" slot and "next" will be free
     * again.
     *
     *  4) `immediate` not set, `next` not set: This is a "NOP" motion command,
     * which does not change anything but let the vehicle platform know that
     * there is someone in charge checking the path following and the way is
     * obstacles free and safe to keep moving.
     *
     *
     * \return false on any error.
     * \sa startWatchdog
     */
    virtual bool motion_execute(
        const std::optional<CVehicleVelCmd::Ptr>& immediate,
        const std::optional<EnqueuedMotionCmd>&   next) = 0;

    /** Stops the vehicle. Different levels of abruptness in the stop can be
     * considered given the emergency condition or not of the command.
     */
    virtual void stop(const STOP_TYPE stopType) = 0;

    virtual void stop_watchdog()
    {
        MRPT_LOG_INFO("Default stop_watchdog() called.");
    }
    virtual void start_watchdog([
        [maybe_unused]] const size_t periodMilliseconds)
    {
        MRPT_LOG_INFO("Default start_watchdog() called.");
    }

    /** @name Event callbacks
     *  @{ */

    virtual void on_nav_end_due_to_error()
    {
        MRPT_LOG_WARN("Default on_nav_end_due_to_error() called.");
    }

    virtual void on_nav_start()
    {
        MRPT_LOG_WARN("Default on_nav_start() event handler called.");
    }

    /** @} */
};

}  // namespace selfdriving
