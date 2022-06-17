/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/WaypointSequencer.h>
#include <selfdriving/algos/render_tree.h>

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
    ASSERT_(config_.globalMapObstacleSource);
    ASSERT_(config_.ptgs.initialized());
    ASSERT_(!config_.ptgs.ptgs.empty());

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

    // The main loop navigation_step() will iterate over waypoints
    MRPT_END
}

void WaypointSequencer::navigation_step()
{
    auto lck = mrpt::lockHelper(navMtx_);

    ASSERTMSG_(initialized_, "navigation_step() called before initialize()");

    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "WaypointSequencer::navigation_step()");

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
                    "WaypointSequencer::navigation_step(): Navigation "
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
                    "[WaypointSequencer::navigation_step()] Stopping "
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
    // Ignore calls too-close in time, e.g. from the navigation_step()
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

    // Checks whether we need to launch a new A* path planner:
    check_have_to_replan();

    // Checks whether the A* planner finished and we have to send a new active
    // trajectory to the path tracker:
    check_new_planner_output();

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
    auto& _ = innerState_;

    std::optional<waypoint_idx_t> firstWpIdx;

    for (size_t i = 0; i < _.waypointNavStatus.waypoints.size(); i++)
    {
        const auto& wp = _.waypointNavStatus.waypoints.at(i);
        if (wp.reached) continue;

        firstWpIdx = i;

        if (!wp.allowSkip) break;
    }
    ASSERT_(firstWpIdx.has_value());

    return *firstWpIdx;
}

WaypointSequencer::PathPlannerOutput WaypointSequencer::path_planner_function(
    WaypointSequencer::PathPlannerInput ppi)
{
    const double RRT_BBOX_MARGIN = 4.0;  // [meters]

    mrpt::math::TBoundingBoxf bbox;

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin =
            mrpt::math::TPoint3Df(RRT_BBOX_MARGIN, RRT_BBOX_MARGIN, .0);
        const auto ptStart = mrpt::math::TPoint3Df(
            ppi.pi.stateStart.pose.x, ppi.pi.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(
            ppi.pi.stateGoal.pose.x, ppi.pi.stateGoal.pose.y, 0);

        bbox.min = ptStart;
        bbox.max = ptStart;
        bbox.updateWithPoint(ptStart - bboxMargin);
        bbox.updateWithPoint(ptStart + bboxMargin);
        bbox.updateWithPoint(ptGoal - bboxMargin);
        bbox.updateWithPoint(ptGoal + bboxMargin);
    }

    ppi.pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
    ppi.pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Start pose: "
        << ppi.pi.stateStart.pose.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Goal pose : "
        << ppi.pi.stateGoal.pose.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] World bbox: "
        << ppi.pi.worldBboxMin.asString() << " - "
        << ppi.pi.worldBboxMax.asString());

    // Do the path planning :
    selfdriving::TPS_Astar planner;

    // time profiler:
    planner.profiler_.enable(false);

    // ~~~~~~~~~~~~~~
    // Add cost maps
    // ~~~~~~~~~~~~~~

    selfdriving::CostMapParameters cmP;
    cmP.resolution                 = 0.05;
    cmP.preferredClearanceDistance = 1.0;  // [m]

    // cost maps: from obstacles
    // ============================
    // TODO: Make static list instead of recreating each time?
    planner.costEvaluators_.clear();

    planner.costEvaluators_.push_back(
        selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
            *config_.globalMapObstacleSource->obstacles(), cmP));

    planner.costEvaluators_.push_back(
        selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
            *config_.localSensedObstacleSource->obstacles(), cmP));

    // cost map #2: prefer to go thru waypoints
    // =============
    MRPT_TODO("Add this cost map");

    // ~~~~~~~~~~~~~~~~~~
    // Obstacles sources
    // ~~~~~~~~~~~~~~~~~~
    if (config_.globalMapObstacleSource)
        ppi.pi.obstacles.push_back(config_.globalMapObstacleSource);

    if (config_.localSensedObstacleSource)
        ppi.pi.obstacles.push_back(config_.localSensedObstacleSource);

    // verbosity level:
    planner.setMinLoggingLevel(this->getMinLoggingLevel());

    planner.params_ = config_.plannerParams;
    {
        std::stringstream ss;
        planner.params_.as_yaml().printAsYAML(ss);
        MRPT_LOG_DEBUG_STREAM(
            "[path_planner_function] A* planner parameters:\n"
            << ss.str());
    }

    // PTGs:
    ppi.pi.ptgs = config_.ptgs;

    // ========== ACTUAL A* PLANNING ================
    PathPlannerOutput ret;
    ret.po = planner.plan(ppi.pi);
    // ================================================

    return ret;
}

void WaypointSequencer::enqueue_path_planner_towards(
    const waypoint_idx_t targetWpIdx)
{
    auto& _ = innerState_;

    MRPT_LOG_DEBUG_STREAM(
        "enqueue_path_planner_towards() called with targetWpIdx="
        << targetWpIdx);

    // ----------------------------------
    // prepare planner request:
    // ----------------------------------
    PathPlannerInput ppi;

    // Starting pose and velocity:
    // The current one plus a bit ahead in the future?
    // ---------------------------------------------------
    MRPT_TODO("Add some pose delta to account for the computation time?");
    ppi.pi.stateStart.pose = lastVehicleLocalization_.pose;
    ppi.pi.stateStart.vel  = lastVehicleOdometry_.odometryVelocityLocal.rotated(
        ppi.pi.stateGoal.pose.phi);

    ASSERT_LT_(targetWpIdx, _.waypointNavStatus.waypoints.size());
    const auto& wp          = _.waypointNavStatus.waypoints.at(targetWpIdx);
    ppi.pi.stateGoal.pose.x = wp.target.x;
    ppi.pi.stateGoal.pose.y = wp.target.y;

    if (wp.targetHeading.has_value())
    {
        // assign heading at target:
        ppi.pi.stateGoal.pose.phi = wp.targetHeading.value();
    }
    else
    {
        MRPT_TODO("Handle no preferred heading");
    }

    // speed at target:
    // ppi.pi.stateGoal.vel;
    MRPT_TODO("Handle speed at target waypoint");

    // ----------------------------------
    // send it for running of the worker thread:
    // ----------------------------------
    _.pathPlannerFuture = pathPlannerPool_.enqueue(
        &WaypointSequencer::path_planner_function, this, ppi);
    _.pathPlannerTarget = targetWpIdx;
}

void WaypointSequencer::check_new_planner_output()
{
    auto& _ = innerState_;

    if (!_.pathPlannerFuture.valid()) return;

    if (std::future_status::ready !=
        _.pathPlannerFuture.wait_for(std::chrono::milliseconds(0)))
        return;

    const auto result = _.pathPlannerFuture.get();

    if (!result.po.success)
    {
        MRPT_LOG_WARN("A* failed to plan towards the target!");
        return;
    }

    if (config_.vizSceneToModify) send_planner_output_to_viz(result.po);

    const auto plannedPath =
        result.po.motionTree.backtrack_path(result.po.goalNodeId);
    for (const auto& step : plannedPath)
    {  //
        std::cout << step.asString() << std::endl;
    }
}

void WaypointSequencer::send_planner_output_to_viz(
    const selfdriving::PlannerOutput& po)
{
    RenderOptions ro;
    ro.highlight_path_to_node_id = po.goalNodeId;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    mrpt::opengl::CSetOfObjects::Ptr planViz =
        render_tree(po.motionTree, po.originalInput, ro);
    planViz->setName("astar_plan_result");

    planViz->setLocation(0, 0, 0.01);  // to easy the vis wrt the ground

    // lock:
    if (config_.on_viz_pre_modify) config_.on_viz_pre_modify();

    if (auto glObj = config_.vizSceneToModify->getByName(planViz->getName());
        glObj)
    {
        auto glContainer =
            std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(glObj);
        ASSERT_(glContainer);
        *glContainer = *planViz;
    }
    else
    {
        config_.vizSceneToModify->insert(planViz);
    }

    // unlock:
    if (config_.on_viz_post_modify) config_.on_viz_post_modify();
}
