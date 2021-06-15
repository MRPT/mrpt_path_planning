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

    navigationEndEventSent_ = false;

    std::lock_guard<std::recursive_mutex> csl(navWaypointsMtx_);

    const size_t N = navRequest.waypoints.size();
    ASSERTMSG_(N > 0, "List of waypoints is empty!");

    // reset fields to default:
    waypointNavStatus_ = WaypointStatusSequence();

    waypointNavStatus_.waypoints.resize(N);
    // Copy waypoints fields data, leave status fields to defaults:
    for (size_t i = 0; i < N; i++)
    {
        ASSERT_(navRequest.waypoints[i].isValid());
        waypointNavStatus_.waypoints[i].Waypoint::operator=(
            navRequest.waypoints[i]);
    }
    waypointNavStatus_.timestamp_nav_started = mrpt::Clock::now();

    // new state:
    navigationState_ = NavState::NAVIGATING;
    navErrorReason_  = NavErrorReason();

    // Reset the bad navigation alarm:
    badNavAlarmMinDistTarget_   = std::numeric_limits<double>::max();
    badNavAlarmLastMinDistTime_ = mrpt::Clock::now();

    MRPT_LOG_DEBUG_STREAM(
        "requestNavigation() called, navigation plan:\n"
        << waypointNavStatus_.getAsText());

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

    const NavState prevState = navigationState_;
    switch (navigationState_)
    {
        case NavState::IDLE:
        case NavState::SUSPENDED:
            if (lastNavigationState_ == NavState::NAVIGATING)
            {
                MRPT_LOG_INFO(
                    "WaypointSequencer::navigationStep(): Navigation "
                    "stopped.");
            }
            break;

        case NavState::NAV_ERROR:
            // Send end-of-navigation event:
            if (lastNavigationState_ == NavState::NAVIGATING &&
                navigationState_ == NavState::NAV_ERROR)
            {
                pendingEvents_.emplace_back([this]() {
                    ASSERT_(config_.vehicleMotionInterface);
                    config_.vehicleMotionInterface->on_nav_end_due_to_error();
                });
            }

            // If we just arrived at this state, stop the robot:
            if (lastNavigationState_ == NavState::NAVIGATING)
            {
                MRPT_LOG_ERROR(
                    "[WaypointSequencer::navigationStep()] Stopping "
                    "navigation "
                    "due to a NavState::NAV_ERROR state!");

                if (config_.vehicleMotionInterface)
                {
                    config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
                    config_.vehicleMotionInterface->stop_watchdog();
                }
            }
            break;

        case NavState::NAVIGATING:
            try
            {
                impl_navigation_step();
            }
            catch (const std::exception& e)
            {
                navigationState_ = NavState::NAV_ERROR;
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
    navigationState_ = NavState::IDLE;

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

    if (navigationState_ == NavState::SUSPENDED)
        navigationState_ = NavState::NAVIGATING;
}
void WaypointSequencer::suspend()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "suspend() called before initialize()");

    MRPT_LOG_DEBUG("WaypointSequencer::suspend() called.");

    if (navigationState_ == NavState::NAVIGATING)
    {
        navigationState_ = NavState::SUSPENDED;

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

    if (navigationState_ == NavState::NAV_ERROR)
    {
        navigationState_ = NavState::IDLE;
        navErrorReason_  = NavErrorReason();
    }
}

WaypointStatusSequence WaypointSequencer::waypoint_nav_status() const
{
    // Make sure the data structure is not under modification:
    auto                   lck = mrpt::lockHelper(navWaypointsMtx_);
    WaypointStatusSequence ret = waypointNavStatus_;

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
            navigationState_           = NavState::NAV_ERROR;
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
    latestPoses_.insert(
        lastVehicleLocalization_.timestamp, lastVehicleLocalization_.pose);
    latestOdomPoses_.insert(
        lastVehicleOdometry_.timestamp, lastVehicleOdometry_.odometry);

    // Purge old ones:
    while (latestPoses_.size() > 1 &&
           mrpt::system::timeDifference(
               latestPoses_.begin()->first, latestPoses_.rbegin()->first) >
               PREVIOUS_POSES_MAX_AGE)
    { latestPoses_.erase(latestPoses_.begin()); }
    while (latestOdomPoses_.size() > 1 &&
           mrpt::system::timeDifference(
               latestOdomPoses_.begin()->first,
               latestOdomPoses_.rbegin()->first) > PREVIOUS_POSES_MAX_AGE)
    { latestOdomPoses_.erase(latestOdomPoses_.begin()); }
}

void WaypointSequencer::impl_navigation_step()
{
    if (lastNavigationState_ != NavState::NAVIGATING)
        internal_on_start_new_navigation();

    // Get current robot kinematic state:
    update_robot_kinematic_state();

    // Have we reached the target location
    // TODO... here?

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

    latestPoses_.clear();  // Clear cache of last poses.
    latestOdomPoses_.clear();

    // Have we just started the navigation?
    if (lastNavigationState_ == NavState::IDLE)
    {
        pendingEvents_.emplace_back([this]() {
            ASSERT_(config_.vehicleMotionInterface);
            config_.vehicleMotionInterface->on_nav_start();
        });
    }
}
