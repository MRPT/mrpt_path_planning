/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/EnqueuedMotionCmd.h>
#include <mpp/data/VehicleLocalizationState.h>
#include <mpp/data/VehicleOdometryState.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mpp
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
class VehicleMotionInterface : public mrpt::system::COutputLogger,
                               public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(VehicleMotionInterface)

   public:
    VehicleMotionInterface()
        : mrpt::system::COutputLogger("VehicleMotionInterface")
    {
    }
    virtual ~VehicleMotionInterface();

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
     * \sa startWatchdog(), supports_enqeued_motions()
     */
    virtual bool motion_execute(
        const std::optional<CVehicleVelCmd::Ptr>& immediate,
        const std::optional<EnqueuedMotionCmd>&   next) = 0;

    /** Reimplement to return true if enqueued motions are supported in
     * motion_execute()
     */
    virtual bool supports_enqeued_motions() const { return false; }

    /** Reimplement to return true if an enqueued motions has been received from
     * motion_execute() and it is still waiting to be executed, or false
     * otherwise (no action enqeued, or it was already triggered and executed).
     */
    virtual bool enqeued_motion_pending() const { return false; }

    /** Reimplement to return true if an enqueued motions has been received from
     * motion_execute() and it has timed out, or false
     * otherwise (no action enqeued, or it was already triggered correctly).
     */
    virtual bool enqeued_motion_timed_out() const { return false; }

    /** Reimplement to return the exact odometry value at the precise moment
     *  that the last enqueued motion was executed by the vehicle motion
     * controller. The `optional<>` should be empty only if no enqueued motion
     * has been executed yet.
     */
    virtual std::optional<VehicleOdometryState>
        enqued_motion_last_odom_when_triggered() const
    {
        return {};
    }

    /** Stops the vehicle. Different levels of abruptness in the stop can be
     * considered given the emergency condition or not of the command.
     */
    virtual void stop(const STOP_TYPE stopType) = 0;

    virtual void stop_watchdog()
    {
        MRPT_LOG_INFO("Default stop_watchdog() called.");
    }
    virtual void start_watchdog(
        [[maybe_unused]] const size_t periodMilliseconds)
    {
        MRPT_LOG_INFO("Default start_watchdog() called.");
    }

    /** Returns clockwall time (UNIX timestamp as double) for real robots
     * [default], simulated time in simulators. */
    virtual double robot_time() const { return mrpt::Clock::nowDouble(); }

    /** @name Event callbacks
     *  @{ */

    /**
     * @brief Callback if current navigation ended due to some error
     */
    virtual void on_nav_end_due_to_error()
    {
        MRPT_LOG_WARN("Default on_nav_end_due_to_error() called.");
    }

    /**
     * @brief Callback upon starting a new waypointsequence navigation
     */
    virtual void on_nav_start()
    {
        MRPT_LOG_WARN("Default on_nav_start() event handler called.");
    }

    /**
     * @brief Callback if navigation ended by an accepted trigger or reached the
     * last specified waypoint
     */
    virtual void on_nav_end()
    {
        MRPT_LOG_WARN("Default on_nav_end() event handler called.");
    }

    /**
     * @brief Callback for when NavEngine cannot make progress to get
     * increasingly closer to the final target during a certain period of time.
     * It may indicate that the high-level path the vehicle is trying to follow
     * is no longer valid due to a blocked way. On your user side, you could
     * call NavEngine::cancel() and/or re-compute an alternative path and issue
     * a new set of navigation waypoints or just report the error to the user.
     *
     * \sa NavEngine::Configuration::timeoutNotGettingCloserGoal
     */
    virtual void on_path_seems_blocked()
    {
        MRPT_LOG_WARN("Default on_path_seems_blocked() event handler called.");
    }

    /**
     * @brief Callback when the NavEngine *predicts* a collision with an
     * obstacle and needed to issue a stop command.
     */
    virtual void on_apparent_collision()
    {
        MRPT_LOG_WARN("Default on_apparent_collision() event handler called.");
    }
    /**
     * @brief Callback upon reaching a waypoint in a waypoint sequeunce.
     * Mostly used for logging
     * @param waypoint_index index of waypoint in the sequence
     * @param reached_skipped whether it was reached(true) or skipped (false)
     */
    virtual void on_waypoint_reached(
        const size_t waypoint_index, bool reached_skipped)
    {
        MRPT_LOG_WARN_FMT(
            "Default on_waypoint_reached() index = %zu event "
            "handler called (event='%s').",
            waypoint_index, reached_skipped ? "reached" : "skipped");
    }

    /**
     * @brief Callback when NavEngine cannot reach a specified target location
     * because there are obstacles at the specified target
     */
    virtual void on_cannot_get_closer_to_blocked_target()
    {
        MRPT_LOG_WARN("Default on_apparent_collision() event handler called.");
    }

    /** @} */
};

}  // namespace mpp
