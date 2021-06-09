/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <selfdriving/algos/WaypointSequencer.h>

using namespace selfdriving;

WaypointSequencer::~WaypointSequencer()
{
    // stop vehicle, etc.
}

void WaypointSequencer::initialize()
{
    MRPT_START
    auto lck = mrpt::lockHelper(m_nav_cs);

    // Check that config_ holds all the required fields:
    ASSERT_(config_.vehicleMotionInterface);
    ASSERT_(config_.obstacleSource);

    // Initialize PTGs...

    initialized_ = true;

    MRPT_END
}

void WaypointSequencer::requestNavigation(const WaypointSequence& navRequest)
{
    MRPT_START
    auto lck = mrpt::lockHelper(m_nav_cs);
    ASSERTMSG_(initialized_, "requestNavigation() called before initialize()");

    m_navigationEndEventSent = false;

    std::lock_guard<std::recursive_mutex> csl(m_nav_waypoints_cs);

    const size_t N = navRequest.waypoints.size();
    ASSERTMSG_(N > 0, "List of waypoints is empty!");

    // reset fields to default:
    m_waypoint_nav_status = WaypointStatusSequence();

    m_waypoint_nav_status.waypoints.resize(N);
    // Copy waypoints fields data, leave status fields to defaults:
    for (size_t i = 0; i < N; i++)
    {
        ASSERT_(navRequest.waypoints[i].isValid());
        m_waypoint_nav_status.waypoints[i].Waypoint::operator=(
            navRequest.waypoints[i]);
    }
    m_waypoint_nav_status.timestamp_nav_started = mrpt::Clock::now();

    // new state:
    navigationState_ = NavState::NAVIGATING;
    m_navErrorReason = NavErrorReason();

    // Reset the bad navigation alarm:
    m_badNavAlarm_minDistTarget   = std::numeric_limits<double>::max();
    m_badNavAlarm_lastMinDistTime = mrpt::Clock::now();

    MRPT_LOG_DEBUG_STREAM(
        "requestNavigation() called, navigation plan:\n"
        << m_waypoint_nav_status.getAsText());

    // The main loop navigationStep() will iterate over waypoints
    MRPT_END
}

void WaypointSequencer::navigationStep()
{
    auto lck = mrpt::lockHelper(m_nav_cs);

    ASSERTMSG_(initialized_, "navigationStep() called before initialize()");

    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "WaypointSequencer::navigationStep()");

    const NavState prevState = navigationState_;
    switch (navigationState_)
    {
        case NavState::IDLE:
        case NavState::SUSPENDED:
            if (lastNavigationState_ == NavState::NAVIGATING)
            {
                MRPT_LOG_INFO(
                    "WaypointSequencer::navigationStep(): Navigation stopped.");
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
                    "[WaypointSequencer::navigationStep()] Stopping navigation "
                    "due to a NavState::NAV_ERROR state!");

                if (config_.vehicleMotionInterface)
                {
                    config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
                    config_.vehicleMotionInterface->stop_watchdog();
                }
            }
            break;

        case NavState::NAVIGATING:
            performNavigationStepNavigating();
            break;
    };

    lastNavigationState_ = prevState;

    dispatchPendingNavEvents();
}

void WaypointSequencer::cancel()
{
    auto lck = mrpt::lockHelper(m_nav_cs);
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
    auto lck = mrpt::lockHelper(m_nav_cs);
    ASSERTMSG_(initialized_, "resume() called before initialize()");

    MRPT_LOG_DEBUG("WaypointSequencer::resume() called.");

    if (navigationState_ == NavState::SUSPENDED)
        navigationState_ = NavState::NAVIGATING;
}
void WaypointSequencer::suspend()
{
    auto lck = mrpt::lockHelper(m_nav_cs);
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

void WaypointSequencer::resetNavError()
{
    auto lck = mrpt::lockHelper(m_nav_cs);
    ASSERTMSG_(initialized_, "resetNavError() called before initialize()");

    if (navigationState_ == NavState::NAV_ERROR)
    {
        navigationState_ = NavState::IDLE;
        m_navErrorReason = NavErrorReason();
    }
}

WaypointStatusSequence WaypointSequencer::getWaypointNavStatus() const
{
    // Make sure the data structure is not under modification:
    auto                   lck = mrpt::lockHelper(m_nav_waypoints_cs);
    WaypointStatusSequence ret = m_waypoint_nav_status;

    return ret;
}

void WaypointSequencer::dispatchPendingNavEvents()
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

void WaypointSequencer::updateCurrentPoseAndSpeeds() {}

void WaypointSequencer::performNavigationStepNavigating()
{
    const auto prevState = navigationState_;
    try
    {
        if (lastNavigationState_ != NavState::NAVIGATING)
        {
            MRPT_LOG_INFO(
                "[WaypointSequencer::navigationStep()] Starting navigation. "
                "Watchdog enabled.\n");

            internal_on_start_new_navigation();
        }

        // Have we just started the navigation?
        if (lastNavigationState_ == NavState::IDLE)
        {
            pendingEvents_.emplace_back([this]() {
                ASSERT_(config_.vehicleMotionInterface);
                config_.vehicleMotionInterface->on_nav_start();
            });
        }

        // Get current robot kinematic state:
        updateCurrentPoseAndSpeeds();

        // Have we reached the target location
        // TODO... here?

        // Check if the target seems to be at reach, but it's clearly
        // occupied by obstacles:
        // TODO... here?
        // m_counter_check_target_is_blocked = 0;
    }
    catch (const std::exception& e)
    {
        navigationState_ = NavState::NAV_ERROR;
        if (m_navErrorReason.error_code == NavError::NONE)
        {
            m_navErrorReason.error_code = NavError::OTHER;
            m_navErrorReason.error_msg =
                std::string("Exception: ") + std::string(e.what());
        }

        MRPT_LOG_ERROR_FMT(
            "[CAbstractNavigator::navigationStep] Exception:\n %s", e.what());

        // Not useful here?
        // if (m_rethrow_exceptions) throw;
    }

    navigationState_ = prevState;
}

void WaypointSequencer::internal_on_start_new_navigation()
{
    ASSERT_(config_.vehicleMotionInterface);

    config_.vehicleMotionInterface->start_watchdog(1000 /*ms*/);

    m_latestPoses.clear();  // Clear cache of last poses.
    m_latestOdomPoses.clear();
}
