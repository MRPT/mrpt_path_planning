/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/TEnumType.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/data/PlannerInput.h>
#include <selfdriving/data/PlannerOutput.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>
#include <selfdriving/data/Waypoints.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>

#include <functional>
#include <list>

namespace selfdriving
{
/** The different statuses for the navigation system. */
enum class NavStatus : uint8_t
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
 * This class is based and extends `mrpt::nav::CAbstractNavigator` with the
 * capability of following a list of waypoints. By default, waypoints are
 * followed one by one, but, refer to \c Waypoint for a discussion on
 * `allow_skip`.
 *
 * How to use:
 *  - \c initialize() must be called before running any actual navigation.
 *
 *  - Callbacks must be provided for interfacing the real (or simulated)
 * robot/vehicle and its sensors. They must be provided as std::function<>
 * instances within \c config_. Note they may be actual functions, or lambdas.
 *
 *  - \c navigation_step() must be called periodically in order to effectively
 * run the navigation. This method will internally call the callbacks to gather
 * sensor data and robot positioning data.
 *
 * This class implements the following state machine (see \c current_state()):
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

        ObstacleSource::Ptr globalMapObstacleSource;
        ObstacleSource::Ptr localSensedObstacleSource;

        TrajectoriesAndRobotShape ptgs;

        /** @} */

        /** @name Parameters
         *  @{ */

        /** In meters. Not present: unlimited */
        std::optional<double> max_distance_to_allow_skip_waypoint;

        /** How many times shall a future waypoint be seen as reachable to skip
         * to it (Default: 1) */
        int min_timesteps_confirm_skip_waypoints = 1;

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

        /** (Default=4.0) Hoy many meters to add at each side
         * (up,down,left,right) of the bbox enclosing the starting and goal pose
         * for each individual call to the A* planner. */
        double planner_bbox_margin = 4.0;

        TPS_Astar_Parameters plannerParams;

        CostEvaluatorCostMap::Parameters           globalCostParameters;
        CostEvaluatorCostMap::Parameters           localCostParameters;
        CostEvaluatorPreferredWaypoint::Parameters preferWaypointsParameters;

        double enqueuedActionsToleranceXY       = 0.05;
        double enqueuedActionsTolerancePhi      = mrpt::DEG2RAD(2.0);
        double enqueuedActionsTimeoutMultiplier = 1.3;

        /** @} */

        /**  \name Visualization callbacks and methods
         *   @{ */

        std::function<void(void)>                    on_viz_pre_modify;
        std::shared_ptr<mrpt::opengl::CSetOfObjects> vizSceneToModify;
        std::function<void(void)>                    on_viz_post_modify;

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
    virtual void request_navigation(const WaypointSequence& navRequest);

    /** This method must be called periodically in order to effectively run the
     * navigation */
    virtual void navigation_step();

    /** Cancel current navegation. */
    virtual void cancel();

    /** Continues with suspended navigation. \sa suspend */
    virtual void resume();

    /** Suspend current navegation. \sa resume */
    virtual void suspend();

    /** Resets a `NAV_ERROR` state back to `IDLE` */
    virtual void reset_nav_error();

    /** Returns the current navigator status. */
    inline NavStatus current_status() const { return navigationStatus_; }

    /** In case of status=NAV_ERROR, this returns the reason for the error.
     * Error status is reseted every time a new navigation starts with
     * a call to navigate(), or when resetNavError() is called.
     */
    inline const NavErrorReason& error_reason() const
    {
        return navErrorReason_;
    }

    /** Get a copy of the control structure which describes the progress status
     * of the waypoint navigation. */
    WaypointStatusSequence waypoint_nav_status() const;

    /** Gets a write-enabled reference to the list of waypoints, simultaneously
     * acquiring the critical section mutex.
     * Caller must call endWaypointsAccess() when done editing the waypoints.
     */
    WaypointStatusSequence& beginWaypointsAccess()
    {
        navMtx_.lock();
        return innerState_.waypointNavStatus;
    }

    /** Must be called after beginWaypointsAccess() */
    void end_waypoints_access() { navMtx_.unlock(); }

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

    struct PathPlannerOutput
    {
        PathPlannerOutput() = default;

        selfdriving::PlannerOutput po;

        /// A copy of the employed costs.
        std::vector<CostEvaluator::Ptr> costEvaluators;
    };

    /** Use the callbacks above and render_tree() to update a visualization
     * with a given plan output */
    void send_planner_output_to_viz(const PathPlannerOutput& ppo);

   protected:
    /** Current and last internal state of navigator: */
    NavStatus      navigationStatus_    = NavStatus::IDLE;
    NavStatus      lastNavigationState_ = NavStatus::IDLE;
    NavErrorReason navErrorReason_;

    std::optional<double> lastNavigationStepEndTime_;

    bool initialized_ = false;

    /** mutex for all navigation methods */
    std::recursive_mutex navMtx_;

    /** Current robot kinematic state; Updated in navigation_step() with a
     * minimum period of MIN_TIME_BETWEEN_POSE_UPDATES.
     */
    VehicleLocalizationState lastVehicleLocalization_;
    VehicleOdometryState     lastVehicleOdometry_;
    double                   lastVehiclePosRobotTime_ = 0;

    /** Events generated during navigation_step(), enqueued to be called at the
     * end of the method execution to avoid user code to change the navigator
     * state. */
    std::list<std::function<void(void)>> pendingEvents_;

    void dispatch_pending_nav_events();

    /** Call to the robot getCurrentPoseAndSpeeds() and updates members
     * m_curPoseVel accordingly.
     * If an error is returned by the user callback, first, it calls
     * robot.stop() ,then throws an std::runtime_error exception. */
    virtual void update_robot_kinematic_state();

    /** The actual action that happens inside navigation_step() for the
     * case of state being NAVIGATING.
     */
    virtual void impl_navigation_step();

    /** Implements the way to waypoint is free function in children classes:
     * `true` must be returned
     * if, according to the information gathered at the last navigation step,
     * there is a free path to
     * the given point; `false` otherwise: if way is blocked or there is
     * missing information, the point is out of range, etc. */
    //    virtual bool impl_waypoint_is_reachable(
    //        const mrpt::math::TPoint2D& wp_local_wrt_robot) const = 0;

    void internal_on_start_new_navigation();

    // Path planning in a parallel thread:
    mrpt::WorkerThreadsPool pathPlannerPool_{
        1 /*Single thread*/, mrpt::WorkerThreadsPool::POLICY_DROP_OLD,
        "path_planner"};

    struct PathPlannerInput
    {
        PathPlannerInput() = default;

        selfdriving::PlannerInput pi;
    };

    // Argument is a copy instead of a const-ref intentionally.
    PathPlannerOutput path_planner_function(PathPlannerInput ppi);

    /** Everything that should be cleared upon a new navigation command. */
    struct CurrentNavInternalState
    {
        CurrentNavInternalState() = default;

        void clear() { *this = CurrentNavInternalState(); }

        /** The latest waypoints navigation command and the up-to-date control
         * status. */
        WaypointStatusSequence waypointNavStatus;

        /** Latest robot poses, updated in navigation_Step() */
        mrpt::poses::CPose2DInterpolator latestPoses, latestOdomPoses;

        std::future<PathPlannerOutput> pathPlannerFuture;
        std::optional<waypoint_idx_t>  pathPlannerTarget;

        /** The final waypoint of the currently under-execution path tracking.
         */
        std::optional<waypoint_idx_t> activeFinalTarget;

        /** Set by check_new_planner_output() */
        PathPlannerOutput                        activePlanOutput;
        MotionPrimitivesTreeSE2::path_t          activePlanPath;
        MotionPrimitivesTreeSE2::edge_sequence_t activePlanPathEdges;
        std::optional<mrpt::graphs::TNodeID>     activePlanNextNodeId;

        // int  counterCheckTargetIsBlocked_ = 0;

        /** For sending an alarm (error event) when it seems that we are not
         * approaching toward the target in a while... */
        double badNavAlarmMinDistTarget_ = std::numeric_limits<double>::max();
        mrpt::Clock::time_point badNavAlarmLastMinDistTime_ =
            mrpt::Clock::now();

        /** Will be false until the navigation end is sent. */
        bool navigationEndEventSent = false;
    };

    /** Navigation state variables, protected by navMtx_ */
    CurrentNavInternalState innerState_;

    /** Checks whether we need to launch a new RRT* path planner */
    void check_have_to_replan();

    /** Checks whether the A* planner finished, then send a new active
     * trajectory to the path tracker */
    void check_new_planner_output();

    /** Checks and send next motion command, or NOP, if we are on track */
    void send_next_motion_cmd_or_nop();

    /** Finds the next waypt index up to which we should find a new RRT*
       plan */
    waypoint_idx_t find_next_waypoint_for_planner();

    /** Enqueues a task in pathPlannerPool_ running path_planner_function() and
     * saving future results into pathPlannerFuture
     */
    void enqueue_path_planner_towards(const waypoint_idx_t target);

#if 0
    bool checkHasReachedTarget(const double targetDist) const override;

    /** The waypoints-specific part of navigation_step() */
    virtual void waypoints_navigation_step();

    bool waypoints_isAligning() const { return m_is_aligning; }

    /** Whether the last timestep was "is_aligning" in a waypoint with heading
     */
    bool                     m_was_aligning{false};
    bool                     m_is_aligning{false};
    mrpt::system::TTimeStamp m_last_alignment_cmd;
#endif
};

}  // namespace selfdriving

MRPT_ENUM_TYPE_BEGIN(selfdriving::NavStatus)
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::IDLE);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::NAVIGATING);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::SUSPENDED);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::NAV_ERROR);
MRPT_ENUM_TYPE_END()
