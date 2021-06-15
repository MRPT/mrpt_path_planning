/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <selfdriving/algos/WaypointSequencer.h>

using namespace selfdriving;

constexpr double MIN_TIME_BETWEEN_POSE_UPDATES = 20e-3;  // [s]
constexpr double PREVIOUS_POSES_MAX_AGE        = 20;  // [s]

WaypointSequencer::~WaypointSequencer()
{
    // stop vehicle, etc.
}

void WaypointSequencer::initialize()
{
    MRPT_START
    auto lck = mrpt::lockHelper(navMtx_);

    // Check that config_ holds all the required fields:
    ASSERT_(config_.vehicleMotionInterface);
    ASSERT_(config_.obstacleSource);
    ASSERT_(config_.ptgs.initialized());

    initialized_ = true;

    MRPT_END
}

void WaypointSequencer::request_navigation(const WaypointSequence& navRequest)
{
    MRPT_START
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "requestNavigation() called before initialize()");

    const size_t N = navRequest.waypoints.size();
    ASSERTMSG_(N > 0, "List of waypoints is empty!");

    // reset fields to default:
    innerState_.clear();

    innerState_.waypointNavStatus.waypoints.resize(N);
    // Copy waypoints fields data, leave status fields to defaults:
    for (size_t i = 0; i < N; i++)
    {
        ASSERT_(navRequest.waypoints[i].isValid());
        innerState_.waypointNavStatus.waypoints[i].Waypoint::operator=(
            navRequest.waypoints[i]);
    }
    innerState_.waypointNavStatus.timestamp_nav_started = mrpt::Clock::now();

    // new state:
    navigationStatus_ = NavStatus::NAVIGATING;
    navErrorReason_   = NavErrorReason();

    MRPT_LOG_DEBUG_STREAM(
        "requestNavigation() called, navigation plan:\n"
        << innerState_.waypointNavStatus.getAsText());

    // The main loop navigationStep() will iterate over waypoints
    MRPT_END
}

void WaypointSequencer::navigation_step()
{
    auto lck = mrpt::lockHelper(navMtx_);

    ASSERTMSG_(initialized_, "navigationStep() called before initialize()");

    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "WaypointSequencer::navigationStep()");

    // Record execution period:
    {
        const double tNow = mrpt::Clock::nowDouble();
        if (lastNavigationStepEndTime_)
            navProfiler_.registerUserMeasure(
                "navigationStep_period", tNow - *lastNavigationStepEndTime_,
                true /*has time units*/);
        lastNavigationStepEndTime_ = tNow;
    }

    const NavStatus prevState = navigationStatus_;
    switch (navigationStatus_)
    {
        case NavStatus::IDLE:
        case NavStatus::SUSPENDED:
            if (lastNavigationState_ == NavStatus::NAVIGATING)
            {
                MRPT_LOG_INFO(
                    "WaypointSequencer::navigationStep(): Navigation "
                    "stopped.");
            }
            break;

        case NavStatus::NAV_ERROR:
            // Send end-of-navigation event:
            if (lastNavigationState_ == NavStatus::NAVIGATING &&
                navigationStatus_ == NavStatus::NAV_ERROR)
            {
                pendingEvents_.emplace_back([this]() {
                    ASSERT_(config_.vehicleMotionInterface);
                    config_.vehicleMotionInterface->on_nav_end_due_to_error();
                });
            }

            // If we just arrived at this state, stop the robot:
            if (lastNavigationState_ == NavStatus::NAVIGATING)
            {
                MRPT_LOG_ERROR(
                    "[WaypointSequencer::navigationStep()] Stopping "
                    "navigation "
                    "due to a NavStatus::NAV_ERROR state!");

                if (config_.vehicleMotionInterface)
                {
                    config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
                    config_.vehicleMotionInterface->stop_watchdog();
                }
            }
            break;

        case NavStatus::NAVIGATING:
            try
            {
                impl_navigation_step();
            }
            catch (const std::exception& e)
            {
                navigationStatus_ = NavStatus::NAV_ERROR;
                if (navErrorReason_.error_code == NavError::NONE)
                {
                    navErrorReason_.error_code = NavError::OTHER;
                    navErrorReason_.error_msg =
                        std::string("Exception: ") + std::string(e.what());
                }
                MRPT_LOG_ERROR_FMT(
                    "[CAbstractNavigator::navigationStep] Exception:\n %s",
                    e.what());
            }
            break;
    };

    lastNavigationState_ = prevState;

    dispatch_pending_nav_events();
}

void WaypointSequencer::cancel()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "cancel() called before initialize()");

    MRPT_LOG_DEBUG("WaypointSequencer::cancel() called.");
    navigationStatus_ = NavStatus::IDLE;

    if (config_.vehicleMotionInterface)
    {
        config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
        config_.vehicleMotionInterface->stop_watchdog();
    }
}
void WaypointSequencer::resume()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "resume() called before initialize()");

    MRPT_LOG_DEBUG("WaypointSequencer::resume() called.");

    if (navigationStatus_ == NavStatus::SUSPENDED)
        navigationStatus_ = NavStatus::NAVIGATING;
}
void WaypointSequencer::suspend()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "suspend() called before initialize()");

    MRPT_LOG_DEBUG("WaypointSequencer::suspend() called.");

    if (navigationStatus_ == NavStatus::NAVIGATING)
    {
        navigationStatus_ = NavStatus::SUSPENDED;

        if (config_.vehicleMotionInterface)
        {
            config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
            config_.vehicleMotionInterface->stop_watchdog();
        }
    }
}

void WaypointSequencer::reset_nav_error()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "resetNavError() called before initialize()");

    if (navigationStatus_ == NavStatus::NAV_ERROR)
    {
        navigationStatus_ = NavStatus::IDLE;
        navErrorReason_   = NavErrorReason();
    }
}

WaypointStatusSequence WaypointSequencer::waypoint_nav_status() const
{
    // Make sure the data structure is not under modification:
    auto lck = mrpt::lockHelper(navMtx_);

    WaypointStatusSequence ret = innerState_.waypointNavStatus;
    return ret;
}

void WaypointSequencer::dispatch_pending_nav_events()
{
    // Invoke pending events:
    for (auto& ev : pendingEvents_)
    {
        try
        {
            ev();
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_ERROR_STREAM("Exception in event handler: " << e.what());
        }
    }
    pendingEvents_.clear();
}

void WaypointSequencer::update_robot_kinematic_state()
{
    // Ignore calls too-close in time, e.g. from the navigationStep()
    // methods of AbstractNavigator and a derived, overriding class.

    // this is clockwall time for real robots, simulated time in simulators.
    const double robotTime = config_.vehicleMotionInterface->robot_time();

    if (lastVehiclePosRobotTime_ >= .0)
    {
        const double lastCallAge = robotTime - lastVehiclePosRobotTime_;
        if (lastCallAge < MIN_TIME_BETWEEN_POSE_UPDATES)
        {
            MRPT_LOG_THROTTLE_DEBUG_FMT(
                5.0,
                "updateCurrentPoseAndSpeeds: ignoring call, since last "
                "call "
                "was only %f ms ago.",
                lastCallAge * 1e3);
            // previous data is still valid: don't query the robot again
            return;
        }
    }

    {
        mrpt::system::CTimeLoggerEntry tle(
            navProfiler_, "updateCurrentPoseAndSpeeds()");

        lastVehicleLocalization_ =
            config_.vehicleMotionInterface->get_localization();

        lastVehicleOdometry_ = config_.vehicleMotionInterface->get_odometry();

        if (!lastVehicleLocalization_.valid)
        {
            navigationStatus_          = NavStatus::NAV_ERROR;
            navErrorReason_.error_code = NavError::EMERGENCY_STOP;
            navErrorReason_.error_msg  = std::string(
                "ERROR: get_localization() failed, stopping robot "
                "and finishing navigation");
            try
            {
                config_.vehicleMotionInterface->stop(STOP_TYPE::EMERGENCY);
            }
            catch (...)
            {
            }
            MRPT_LOG_ERROR(navErrorReason_.error_msg);
            throw std::runtime_error(navErrorReason_.error_msg);
        }
    }
    lastVehiclePosRobotTime_ = robotTime;

    // TODO: Detect a change if frame_id and clear m_latestPoses,
    // m_latestOdomPoses.

    // Append to list of past poses:
    innerState_.latestPoses.insert(
        lastVehicleLocalization_.timestamp, lastVehicleLocalization_.pose);
    innerState_.latestOdomPoses.insert(
        lastVehicleOdometry_.timestamp, lastVehicleOdometry_.odometry);

    // Purge old ones:
    while (innerState_.latestPoses.size() > 1 &&
           mrpt::system::timeDifference(
               innerState_.latestPoses.begin()->first,
               innerState_.latestPoses.rbegin()->first) >
               PREVIOUS_POSES_MAX_AGE)
    { innerState_.latestPoses.erase(innerState_.latestPoses.begin()); }
    while (innerState_.latestOdomPoses.size() > 1 &&
           mrpt::system::timeDifference(
               innerState_.latestOdomPoses.begin()->first,
               innerState_.latestOdomPoses.rbegin()->first) >
               PREVIOUS_POSES_MAX_AGE)
    {
        innerState_.latestOdomPoses.erase(innerState_.latestOdomPoses.begin());
    }
}

void WaypointSequencer::impl_navigation_step()
{
    if (lastNavigationState_ != NavStatus::NAVIGATING)
        internal_on_start_new_navigation();

    // Get current robot kinematic state:
    update_robot_kinematic_state();

    // Checks whether we need to launch a new RRT* path planner:
    check_have_to_replan();

    // Check if the target seems to be at reach, but it's clearly
    // occupied by obstacles:
    // TODO... here?
    // m_counter_check_target_is_blocked = 0;
}

void WaypointSequencer::internal_on_start_new_navigation()
{
    ASSERT_(config_.vehicleMotionInterface);

    MRPT_LOG_INFO("Starting navigation. Watchdog enabled.");

    config_.vehicleMotionInterface->start_watchdog(1000 /*ms*/);

    // Have we just started the navigation?
    if (lastNavigationState_ == NavStatus::IDLE)
    {
        pendingEvents_.emplace_back([this]() {
            ASSERT_(config_.vehicleMotionInterface);
            config_.vehicleMotionInterface->on_nav_start();
        });
    }
}

void WaypointSequencer::check_have_to_replan()
{
    auto& _ = innerState_;

    // We don't have yet neither a running path following, nor a path planner:
    if (!_.activeFinalTarget && !_.pathPlannerTarget)
    {
        // find next target wp:
        auto nextWp = find_next_waypoint_for_planner();

        enqueue_path_planner_towards(nextWp);
    }
}

waypoint_idx_t WaypointSequencer::find_next_waypoint_for_planner()
{
    //
    return 1;
}

WaypointSequencer::PathPlannerOutput WaypointSequencer::path_planner_function(
    WaypointSequencer::PathPlannerInput ppi)
{
    PathPlannerOutput ret;

    return ret;
}

void WaypointSequencer::enqueue_path_planner_towards(
    const waypoint_idx_t target)
{
    auto& _ = innerState_;

    // ----------------------------------
    // prepare planner request:
    // ----------------------------------
    PathPlannerInput ppi;

    // Starting pose and velocity:
    // The current one plus a bit ahead in the future?
    // ---------------------------------------------------
    ppi.pi.stateGoal.pose = lastVehicleLocalization_.pose;
    // ppi.pi.stateGoal.vel =
    // lastVehicleOdometry_.odometryVelocityLocal.rotate();

    // ----------------------------------
    // send it for running of the worker thread:
    // ----------------------------------
    _.pathPlannerFuture = pathPlannerPool_.enqueue(
        &WaypointSequencer::path_planner_function, this, ppi);
    _.pathPlannerTarget = target;
}
