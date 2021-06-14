/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>
#include <selfdriving/data/Waypoints.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>

#include <functional>
#include <list>

namespace selfdriving
{
/** The different states for the navigation system. */
enum class NavState : uint8_t
{
    IDLE = 0,
    NAVIGATING,
    SUSPENDED,
    NAV_ERROR
};

/** Explains the reason for the navigation error. */
enum class NavError : uint8_t
{
    NONE = 0,
    EMERGENCY_STOP,
    CANNOT_REACH_TARGET,
    OTHER
};
struct NavErrorReason
{
    NavErrorReason()       = default;
    NavError    error_code = NavError::NONE;
    std::string error_msg;  //!< Human friendly description of the error
};

/** The top-level interface for users to control the vehicle navigation.
 *
 *  This class is based and extends `mrpt::nav::CAbstractNavigator` with the
 * capability of following a list of waypoints. By default, waypoints are
 * followed one by one, but, refer to \c Waypoint for a discussion on
 * `allow_skip`.
 *
 * How to use:
 *  - \c initialize() must be called before running any actual navigation.
 *  - Callbacks must be provided for interfacing the real (or simulated)
 * robot/vehicle. They are given as arguments to \c initialize().
 *  - \c navigationStep() must be called periodically in order to effectively
 * run the navigation. This method will internally call the callbacks to gather
 * sensor data and robot positioning data.
 *
 * This class implements the following state machine (see \c getCurrentState()):
 * \dot
 *  digraph CAbstractNavigator_States {
 *      IDLE; NAVIGATING; SUSPENDED; NAV_ERROR;
 *      IDLE -> NAVIGATING [ label="requestNavigation()" ];
 *      NAVIGATING -> IDLE [ label="Final target reached" ];
 *      NAVIGATING -> IDLE [ label="cancel()" ];
 *      NAVIGATING -> NAV_ERROR [ label="Upon sensor errors, timeout,..." ];
 *      NAVIGATING -> SUSPENDED [ label="suspend()" ];
 *      SUSPENDED -> NAVIGATING [ label="resume()" ];
 *      NAV_ERROR -> IDLE [ label="resetNavError()" ];
 *  }
 *  \enddot
 *
 * All methods are thread-safe, in the sense that mutexes are internally used
 * to ensure no undefined navigation state is possible if invoking an object of
 * this class from more than one thread.
 *
 */
class WaypointSequencer : public mrpt::system::COutputLogger
{
   public:
    /** ctor */
    WaypointSequencer() : mrpt::system::COutputLogger("WaypointSequencer") {}

    /** dtor */
    virtual ~WaypointSequencer();

    /** \name Initialization and parameters
     * @{ */

    struct Configuration
    {
        Configuration() = default;

        /** @name Mandatory configuration fields; must fill before initialize()
         *  @{ */
        VehicleMotionInterface::Ptr vehicleMotionInterface;

        ObstacleSource::Ptr obstacleSource;

        TrajectoriesAndRobotShape ptgs;

        /** @} */

        /** @name Parameters
         *  @{ */

        /** In meters. Not present: unlimited */
        std::optional<double> max_distance_to_allow_skip_waypoint;

        /** How many times shall a future waypoint be seen as reachable to skip
         * to it (Default: 1) */
        int min_timesteps_confirm_skip_waypoints = 1;

        /** [rad] Angular error tolerance for waypoints with an assigned heading
         * (Default: 5 deg) */
        // double waypoint_angle_tolerance = mrpt::DEG2RAD(5.0);

        /** >=0 number of waypoints to forward to the underlying navigation
         * engine, to ease obstacles avoidance when a waypoint is blocked
         * (Default=2). */
        int multitarget_look_ahead = 2;

        /** Default value=0, means use the "targetAllowedDistance" passed by the
         * user in the navigation request. */
        double dist_to_target_for_sending_event{0};

        /** navigator timeout (seconds) [Default=30 sec] */
        double alarm_seems_not_approaching_target_timeout{30};

        /** (Default value=0.6) When closer than this distance, check if the
         * target is blocked to abort navigation with an error. */
        double dist_check_target_is_blocked{0.6};

        /** (Default=3) How many steps should the condition for
         * dist_check_target_is_blocked be fulfilled to raise an event */
        int hysteresis_check_target_is_blocked{3};

        /** @} */
    };

    /** Must be called before any other navigation command, and after filling in
     *  all the required data into \c config_
     */
    virtual void initialize();

    Configuration config_;

    /** @} */

    /** \name Waypoint navigation control API
     * @{ */

    /** Waypoint navigation request. This immediately cancels any other previous
     * on-going navigation.
     */
    virtual void requestNavigation(const WaypointSequence& navRequest);

    /** This method must be called periodically in order to effectively run the
     * navigation */
    virtual void navigationStep();

    /** Cancel current navegation. */
    virtual void cancel();

    /** Continues with suspended navigation. \sa suspend */
    virtual void resume();

    /** Suspend current navegation. \sa resume */
    virtual void suspend();

    /** Resets a `NAV_ERROR` state back to `IDLE` */
    virtual void resetNavError();

    /** Returns the current navigator state. */
    inline NavState getCurrentState() const { return navigationState_; }

    /** In case of state=NAV_ERROR, this returns the reason for the error.
     * Error state is reseted every time a new navigation starts with
     * a call to navigate(), or when resetNavError() is called.
     */
    inline const NavErrorReason& getErrorReason() const
    {
        return m_navErrorReason;
    }

    /** Get a copy of the control structure which describes the progress status
     * of the waypoint navigation. */
    WaypointStatusSequence getWaypointNavStatus() const;

    /** Gets a write-enabled reference to the list of waypoints, simultaneously
     * acquiring the critical section mutex.
     * Caller must call endWaypointsAccess() when done editing the waypoints.
     */
    WaypointStatusSequence& beginWaypointsAccess()
    {
        m_nav_waypoints_cs.lock();
        return m_waypoint_nav_status;
    }

    /** Must be called after beginWaypointsAccess() */
    void endWaypointsAccess() { m_nav_waypoints_cs.unlock(); }

    /** Publicly available time profiling object. Default: disabled */
    mrpt::system::CTimeLogger navProfiler_{
        true /*enabled*/, "WaypointSequencer"};

    /** @}*/

#if 0
    /** Returns `true` if, according to the information gathered at the last
     * navigation step,
     * there is a free path to the given point; `false` otherwise: if way is
     * blocked or there is missing information,
     * the point is out of range for the existing PTGs, etc. */
    bool isRelativePointReachable(
        const mrpt::math::TPoint2D& wp_local_wrt_robot) const;
#endif

   protected:
    /** Current and last internal state of navigator: */
    NavState       navigationState_     = NavState::IDLE;
    NavState       lastNavigationState_ = NavState::IDLE;
    NavErrorReason m_navErrorReason;

    bool initialized_ = false;

    /** mutex for all navigation methods */
    std::recursive_mutex m_nav_cs;

    /** Current robot pose (updated in navigationStep() ) */
    VehicleLocalizationState lastVehicleLocalization_;
    VehicleOdometryState     lastVehicleOdometry_;
    double                   lastVehiclePosRobotTime_ = 0;

    /** Latest robot poses (updated in CAbstractNavigator::navigationStep() ) */
    mrpt::poses::CPose2DInterpolator latestPoses_, latestOdomPoses_;

    /** For sending an alarm (error event) when it seems that we are not
     * approaching toward the target in a while... */
    double                  m_badNavAlarm_minDistTarget;
    mrpt::Clock::time_point m_badNavAlarm_lastMinDistTime;

    /** Events generated during navigationStep(), enqueued to be called at the
     * end of the method execution to avoid user code to change the navigator
     * state. */
    std::list<std::function<void(void)>> pendingEvents_;

    void dispatchPendingNavEvents();

    /** Call to the robot getCurrentPoseAndSpeeds() and updates members
     * m_curPoseVel accordingly.
     * If an error is returned by the user callback, first, it calls
     * robot.stop() ,then throws an std::runtime_error exception. */
    virtual void updateCurrentPoseAndSpeeds();

    /** Factorization of the part inside navigationStep(), for the case of state
     * being NAVIGATING.
     * Performs house-hold tasks like raising events in case of starting/ending
     * navigation, timeout reaching destination, etc.
     * `call_virtual_nav_method` can be set to false to avoid calling the
     * virtual method performNavigationStep()
     */
    virtual void performNavigationStepNavigating();

    /** Will be false until the navigation end is sent, and it is reset with
     * each new command */
    bool m_navigationEndEventSent          = false;
    int  m_counter_check_target_is_blocked = 0;

    /** The latest waypoints navigation command and the up-to-date control
     * status. */
    WaypointStatusSequence m_waypoint_nav_status;
    std::recursive_mutex   m_nav_waypoints_cs;

    /** Implements the way to waypoint is free function in children classes:
     * `true` must be returned
     * if, according to the information gathered at the last navigation step,
     * there is a free path to
     * the given point; `false` otherwise: if way is blocked or there is
     * missing information, the point is out of range, etc. */
    //    virtual bool impl_waypoint_is_reachable(
    //        const mrpt::math::TPoint2D& wp_local_wrt_robot) const = 0;

    void internal_on_start_new_navigation();

#if 0
    bool checkHasReachedTarget(const double targetDist) const override;

    /** The waypoints-specific part of navigationStep() */
    virtual void waypoints_navigationStep();

    bool waypoints_isAligning() const { return m_is_aligning; }

    /** Whether the last timestep was "is_aligning" in a waypoint with heading
     */
    bool                     m_was_aligning{false};
    bool                     m_is_aligning{false};
    mrpt::system::TTimeStamp m_last_alignment_cmd;
#endif
};

}  // namespace selfdriving
