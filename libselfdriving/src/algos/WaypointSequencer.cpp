/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/WaypointSequencer.h>
#include <selfdriving/algos/refine_trajectory.h>
#include <selfdriving/algos/render_tree.h>
#include <selfdriving/algos/render_vehicle.h>
#include <selfdriving/algos/trajectories.h>
#include <selfdriving/algos/viz.h>

using namespace selfdriving;

constexpr double MIN_TIME_BETWEEN_POSE_UPDATES = 20e-3;  // [s]
constexpr double PREVIOUS_POSES_MAX_AGE        = 20;  // [s]

WaypointSequencer::~WaypointSequencer()
{
    // stop vehicle, etc.
}

void WaypointSequencer::Configuration::loadFrom(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, planner_bbox_margin);
    MCP_LOAD_REQ(c, enqueuedActionsToleranceXY);
    MCP_LOAD_REQ_DEG(c, enqueuedActionsTolerancePhi);
    MCP_LOAD_REQ(c, enqueuedActionsTimeoutMultiplier);
    MCP_LOAD_REQ(c, minEdgeTimeToRefinePath);
    MCP_LOAD_REQ(c, lookAheadImmediateCollisionChecking);
}

mrpt::containers::yaml WaypointSequencer::Configuration::saveTo() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    MCP_SAVE(c, planner_bbox_margin);
    MCP_SAVE(c, enqueuedActionsToleranceXY);
    MCP_SAVE_DEG(c, enqueuedActionsTolerancePhi);
    MCP_SAVE(c, enqueuedActionsTimeoutMultiplier);
    MCP_SAVE(c, minEdgeTimeToRefinePath);
    MCP_SAVE(c, lookAheadImmediateCollisionChecking);

    return c;
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

    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "navigation_step()");

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
    innerState_.active_plan_reset();

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

    MRPT_LOG_THROTTLE_DEBUG_STREAM(
        1.0,
        "updateCurrentPoseAndSpeeds:"
        "\nLocalization="
            << lastVehicleLocalization_.pose << "\n Odometry    : "
            << lastVehicleOdometry_.odometry << "\n Odometry vel Local: "
            << lastVehicleOdometry_.odometryVelocityLocal.asString()
            << "\n Odometry vel global: "
            << lastVehicleOdometry_.odometryVelocityLocal
                   .rotated(lastVehicleOdometry_.odometry.phi)
                   .asString());

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
    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "impl_navigation_step");

    if (lastNavigationState_ != NavStatus::NAVIGATING)
        internal_on_start_new_navigation();

    // Get current robot kinematic state:
    update_robot_kinematic_state();

    // Check for immediate collisions:
    check_immediate_collision();

    // Checks whether we need to launch a new A* path planner:
    check_have_to_replan();

    // Checks whether the A* planner finished and we have to send a new active
    // trajectory to the path tracker:
    check_new_planner_output();

    // Check if the target seems to be at reach, but it's clearly
    // occupied by obstacles:
    // TODO... here?
    // m_counter_check_target_is_blocked = 0;

    // Send actual motion command, if needed, or a NOP if we are safely on track
    send_next_motion_cmd_or_nop();

    send_current_state_to_viz();  // optional debug viz
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

void WaypointSequencer::check_immediate_collision()
{
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.check_immediate_collision");

    const unsigned int NUM_STEPS             = 3;
    const unsigned int NUM_CLOSEST_OBSTACLES = 40;

    auto& _ = innerState_;

    if (!config_.localSensedObstacleSource) return;

    auto obs = config_.localSensedObstacleSource->obstacles();

    MRPT_LOG_WARN_STREAM("OBS: " << obs->size());

    if (!obs || obs->empty()) return;

    // Extrapolate the current motion into the future:
    const auto globalPos = lastVehicleLocalization_.pose;
    const auto localVel  = lastVehicleOdometry_.odometryVelocityLocal;

    const auto& xs = obs->getPointsBufferRef_x();
    const auto& ys = obs->getPointsBufferRef_y();

    _.collisionCheckingPosePrediction =
        globalPos + localVel * config_.lookAheadImmediateCollisionChecking;

    bool collision = false;

    for (unsigned int i = 0; i < NUM_STEPS && !collision; i++)
    {
        const double dt = (static_cast<double>(i) / (NUM_STEPS - 1)) *
                          config_.lookAheadImmediateCollisionChecking;

        const auto predictedPose = globalPos + localVel * dt;

        for (const auto& ptg : config_.ptgs.ptgs)
        {
            std::vector<size_t> idxs;
            std::vector<float>  distSq;
            obs->kdTreeNClosestPoint2DIdx(
                predictedPose.x, predictedPose.y, NUM_CLOSEST_OBSTACLES, idxs,
                distSq);

            for (size_t ptIdx : idxs)
            {
                const auto localPt =
                    predictedPose.inverseComposePoint({xs[ptIdx], ys[ptIdx]});
                const bool collide =
                    ptg->isPointInsideRobotShape(localPt.x, localPt.y);

                if (collide) collision = true;
            }
            if (collision) break;
        }
    }

    if (collision)
    {
        //
        MRPT_LOG_WARN_STREAM("Collision predicted ahead!");
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
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.find_next_waypoint_for_planner");

    auto& _ = innerState_;

    ASSERT_(!_.waypointNavStatus.waypoints.empty());
    const auto& wps = _.waypointNavStatus.waypoints;

    std::optional<waypoint_idx_t> firstWpIdx;

    for (waypoint_idx_t i = 0; i < wps.size(); i++)
    {
        const auto& wp = wps.at(i);
        if (wp.reached) continue;

        firstWpIdx = i;

        if (!wp.allowSkip) break;
    }
    ASSERT_(firstWpIdx.has_value());

    // Raise a warning if the wp is the last one and has not a speed of zero,
    // i.e. the vehicle will keep moving afterwards. It might be desired by the
    // user, so do not abort/error but at least emit a warning:
    if (const auto& wp = wps.at(*firstWpIdx);
        *firstWpIdx + 1 == wps.size() && wp.speedRatio != 0)
    {
        MRPT_LOG_WARN_STREAM(
            "Selecting *last* waypoint #"
            << (*firstWpIdx + 1)
            << " which does not have a final speed of zero: the vehicle will "
               "not stop there. Waypoint: "
            << wp.getAsText());
    }

    return *firstWpIdx;
}

WaypointSequencer::PathPlannerOutput WaypointSequencer::path_planner_function(
    WaypointSequencer::PathPlannerInput ppi)
{
    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "path_planner_function");

    const double BBOX_MARGIN = config_.planner_bbox_margin;  // [meters]

    mrpt::math::TBoundingBoxf bbox;

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin =
            mrpt::math::TPoint3Df(BBOX_MARGIN, BBOX_MARGIN, .0);
        const auto ptStart = mrpt::math::TPoint3Df(
            ppi.pi.stateStart.pose.x, ppi.pi.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(
            ppi.pi.stateGoal.asSE2KinState().pose.x,
            ppi.pi.stateGoal.asSE2KinState().pose.y, 0);

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
        "[path_planner_function] Start state: "
        << ppi.pi.stateStart.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Goal state : " << ppi.pi.stateGoal.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] World bbox: "
        << ppi.pi.worldBboxMin.asString() << " - "
        << ppi.pi.worldBboxMax.asString());

    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Using VehicleInterface class: "
        << config_.vehicleMotionInterface->GetRuntimeClass()->className);

    // Do the path planning :
    selfdriving::TPS_Astar planner;

    // time profiler:
    planner.attachExternalProfiler_(navProfiler_);

    // ~~~~~~~~~~~~~~
    // Add cost maps
    // ~~~~~~~~~~~~~~
    // TODO: Make static list instead of recreating each time?
    planner.costEvaluators_.clear();

    // cost map: prefer to go thru waypoints
    // =========================================
    {
        std::vector<mrpt::math::TPoint2D> lstPts;
        for (const auto& wp : innerState_.waypointNavStatus.waypoints)
        {
            if (wp.reached) continue;
            lstPts.emplace_back(wp.target);
        }

        if (!lstPts.empty())
        {
            auto cmWps = selfdriving::CostEvaluatorPreferredWaypoint::Create();
            cmWps->params_ = config_.preferWaypointsParameters;
            cmWps->setPreferredWaypoints(lstPts);

            planner.costEvaluators_.push_back(cmWps);
        }
    }

    // cost maps: from obstacles
    // ============================

    if (config_.globalMapObstacleSource)
    {
        if (auto obs = config_.globalMapObstacleSource->obstacles();
            obs && !obs->empty())
        {
            planner.costEvaluators_.push_back(
                selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                    *obs, config_.globalCostParameters,
                    ppi.pi.stateStart.pose));
        }
    }

    if (config_.localSensedObstacleSource)
    {
        if (auto obs = config_.localSensedObstacleSource->obstacles();
            obs && !obs->empty())
        {
            planner.costEvaluators_.push_back(
                selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                    *obs, config_.localCostParameters, ppi.pi.stateStart.pose));
        }
    }

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

    // Insert custom progress callback for the GUI, if enabled:
    if (config_.vizSceneToModify)
    {
        planner.progressCallback_ = [this](const ProgressCallbackData& pcd) {
            MRPT_LOG_INFO_STREAM(
                "[progressCallback] bestCostFromStart: "
                << pcd.bestCostFromStart
                << " bestCostToGoal: " << pcd.bestCostToGoal
                << " bestPathLength: " << pcd.bestPath.size());

            ASSERT_(pcd.tree);
            ASSERT_(pcd.originalPlanInput);
            ASSERT_(pcd.costEvaluators);

            send_path_to_viz(
                *pcd.tree, pcd.bestFinalNode, *pcd.originalPlanInput,
                *pcd.costEvaluators);
        };
    }

    mrpt::system::CTimeLoggerEntry tle2(
        navProfiler_, "path_planner_function.a_star");

    // ========== ACTUAL A* PLANNING ================
    PathPlannerOutput ret;
    ret.po = planner.plan(ppi.pi);
    // ================================================

    tle2.stop();

    // Keep a copy of the costs, for reference of the caller,
    // visualization,...
    ret.costEvaluators = planner.costEvaluators_;

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
        ppi.pi.stateStart.pose.phi);

    ASSERT_LT_(targetWpIdx, _.waypointNavStatus.waypoints.size());
    const auto& wp = _.waypointNavStatus.waypoints.at(targetWpIdx);

    // waypoint => pose or point:
    if (wp.targetHeading.has_value())
    {
        ppi.pi.stateGoal.state = mrpt::math::TPose2D(
            wp.target.x, wp.target.y, wp.targetHeading.value());
    }
    else
    {
        ppi.pi.stateGoal.state = mrpt::math::TPoint2D(wp.target.x, wp.target.y);
    }

    // speed at target:
    // ppi.pi.stateGoal.vel
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
        MRPT_LOG_INFO(
            "A* did not complete a plan towards the target, it only had time "
            "for a partial solution");
    }

    if (config_.vizSceneToModify) send_planner_output_to_viz(result);

    // TODO: anything to do with current activePath before overwritting it?
    _.activePlanOutput = std::move(result);
    _.active_plan_reset();

    {
        auto [path, edges] = _.activePlanOutput.po.motionTree.backtrack_path(
            _.activePlanOutput.po.bestNodeId);

        // Correct PTG arguments according to the final actual poses.
        // Needed to correct for lattice approximations:
        refine_trajectory(path, edges, config_.ptgs);

        // std::list -> std::vector for convenience:
        _.activePlanPath.clear();
        for (const auto& node : path) _.activePlanPath.push_back(node);

        // std::list -> std::vector for convenience:
        _.activePlanPathEdges.clear();
        for (const auto& edge : edges) _.activePlanPathEdges.push_back(edge);

#if 0
        const auto traj = selfdriving::plan_to_trajectory(
            _.activePlanPathEdges, config_.ptgs);
        selfdriving::save_to_txt(traj, "traj.txt");
#endif
    }

    for (const auto& step : _.activePlanPath)
    {  //
        std::cout << step.asString() << std::endl;
    }
}

void WaypointSequencer::send_next_motion_cmd_or_nop()
{
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.send_next_motion_cmd_or_nop");

    using namespace mrpt;  // "_deg"

    auto& _ = innerState_;

    // No active path planning?
    if (_.activePlanPath.empty()) return;

    // Following error due to enqueued motion time out?
    if (config_.vehicleMotionInterface->enqeued_motion_timed_out())
    {
        // Abort path planning:
        cancel();
        MRPT_TODO("Re-plan and/or send error event out?");
        return;
    }

    // First edge of a plan?
    if (!_.activePlanEdgeIndex.has_value())
    {
        ASSERT_EQUAL_(
            _.activePlanPath.size(), _.activePlanPathEdges.size() + 1);

        _.activePlanEdgeIndex = 0;  // first edge

        // save odometry at the beginning of the first edge:
        ASSERT_LT_(
            mrpt::system::timeDifference(
                lastVehicleOdometry_.timestamp, mrpt::Clock::now()),
            1.0);

        _.activePlanInitOdometry = lastVehicleOdometry_.odometry;

        MRPT_LOG_INFO_STREAM(
            "Starting motion plan with:\n"
            " - odometry    : "
            << lastVehicleOdometry_.odometry << "\n"
            << " - localization: " << lastVehicleLocalization_.pose);
    }

    // Waiting for the end of this edge motion?
    // Must be done *before* the next if() block:
    if (_.activePlanEdgeSentIndex.has_value() &&
        *_.activePlanEdgeSentIndex == *_.activePlanEdgeIndex)
    {
        if (!config_.vehicleMotionInterface->enqeued_motion_pending())
        {
            // We are ready for the next one:
            _.activePlanEdgeIndex.value()++;

            MRPT_LOG_INFO_STREAM(
                "Enqueued motion seems to have been done. Moving to next edge #"
                << *_.activePlanEdgeIndex);
        }
    }

    // Time to send out a new edge:
    if ((!_.activePlanEdgeSentIndex.has_value() ||
         *_.activePlanEdgeSentIndex != *_.activePlanEdgeIndex) &&
        _.activePlanPath.size() > *_.activePlanEdgeIndex + 1)
    {
        const auto& nFirst = _.activePlanPath.at(0);
        const auto& nCurr  = _.activePlanPath.at(*_.activePlanEdgeIndex);
        const auto& nNext  = _.activePlanPath.at(*_.activePlanEdgeIndex + 1);

        std::optional<MotionPrimitivesTreeSE2::node_t> nAfterNext;
        if (_.activePlanPath.size() > *_.activePlanEdgeIndex + 2)
            nAfterNext = _.activePlanPath.at(*_.activePlanEdgeIndex + 2);

        // Mark the next "motion edge" as "sent":
        _.activePlanEdgeSentIndex = *_.activePlanEdgeIndex;

        // If we have at least two actions, use the two actions at a time API:

        // Check if this robot supports enqueued actions:
        const auto supportsEnqueued =
            config_.vehicleMotionInterface->supports_enqeued_motions();
        ASSERT_(supportsEnqueued);  // TODO: Implement adaptor layer

        const auto edge = *_.activePlanPathEdges.at(*_.activePlanEdgeIndex);

        auto& ptg = config_.ptgs.ptgs.at(edge.ptgIndex);
        ptg->updateNavDynamicState(edge.getPTGDynState());

        const mrpt::kinematics::CVehicleVelCmd::Ptr generatedMotionCmd =
            ptg->directionToMotionCommand(edge.ptgPathIndex);

        ASSERT_(generatedMotionCmd);

        // Before changing the dynamic status of the (potentially same one) PTG
        // for the "next" edge, query the PTG for the extra additional motion
        // required for the condPose below:
        std::optional<mrpt::math::TPose2D> poseCondDeltaForTolerance;
        if (1)
        {
            uint32_t stepEnd = 0, stepAfter = 0;
            bool     ok1 = ptg->getPathStepForDist(
                edge.ptgPathIndex, edge.ptgDist, stepEnd);
            bool ok2 = ptg->getPathStepForDist(
                edge.ptgPathIndex,
                edge.ptgDist + config_.enqueuedActionsToleranceXY, stepAfter);
            if (ok1 && ok2)
            {
                poseCondDeltaForTolerance =
                    ptg->getPathPose(edge.ptgPathIndex, stepAfter) -
                    ptg->getPathPose(edge.ptgPathIndex, stepEnd);
            }
        }

        // Next edge motion:
        mrpt::kinematics::CVehicleVelCmd::Ptr generatedMotionCmdAfter;
        if (nAfterNext.has_value())
        {
            const auto nextEdge =
                *_.activePlanPathEdges.at(*_.activePlanEdgeIndex + 1);

            auto& nextPtg = config_.ptgs.ptgs.at(nextEdge.ptgIndex);
            nextPtg->updateNavDynamicState(nextEdge.getPTGDynState());

            generatedMotionCmdAfter =
                nextPtg->directionToMotionCommand(nextEdge.ptgPathIndex);

            ASSERT_(generatedMotionCmdAfter);

            const auto relPoseNext = nNext.pose - nCurr.pose;
            // const auto relPoseAfterNext = nAfterNext->pose - nCurr.pose;

            ASSERT_(_.activePlanInitOdometry.has_value());

            // Convert from the "map" localization frame to "odom" frame:
            auto condPose =
                _.activePlanInitOdometry.value() + (nNext.pose - nFirst.pose);

            // Shift the "condition pose" such that the desired nominal pose is
            // reached within one box of size toleranceXY:
            if (poseCondDeltaForTolerance.has_value())
                condPose = condPose + poseCondDeltaForTolerance.value();

            // Create the motion command and send to the user-provided interface
            // to the vehicle:
            MRPT_LOG_INFO_STREAM(
                "Generating compound motion cmd to move from node ID "
                << nCurr.nodeID_ << " => " << nNext.nodeID_ << " => "
                << nAfterNext->nodeID_
                << "\n CMD1: " << generatedMotionCmd->asString()
                << "\n CMD2: " << generatedMotionCmdAfter->asString()
                << "\n relPoseNext: " << relPoseNext  //
                << "\n CondPose: " << condPose  //
                << "\n ETA: " << edge.estimatedExecTime  //
                << "\n withTolDelta: "
                << (poseCondDeltaForTolerance
                        ? poseCondDeltaForTolerance.value().asString()
                        : std::string("(none)"))  //
                << "\n nNext.pose: " << nNext.pose  //
                << "\n nCurr.pose: " << nCurr.pose  //
            );

            selfdriving::EnqueuedMotionCmd enqMotion;
            enqMotion.nextCmd                = generatedMotionCmdAfter;
            enqMotion.nextCondition.position = condPose;

            enqMotion.nextCondition.tolerance = {
                config_.enqueuedActionsToleranceXY,
                config_.enqueuedActionsToleranceXY,
                config_.enqueuedActionsTolerancePhi};

            enqMotion.nextCondition.timeout =
                std::max(1.0, edge.estimatedExecTime) *
                config_.enqueuedActionsTimeoutMultiplier;

            _.activeEnqueuedConditionForViz = enqMotion.nextCondition;

            // if the immediate cmd was already sent out, skip it and just send
            // the enqueued part:
            if (_.activePlanEdgesSentOut.count(*_.activePlanEdgeIndex) == 0)
            {
                // send both, both are new:
                config_.vehicleMotionInterface->motion_execute(
                    generatedMotionCmd, enqMotion);

                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex);
                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex + 1);
            }
            else
            {
                // Only the enqueued part is new:
                config_.vehicleMotionInterface->motion_execute(
                    std::nullopt, enqMotion);

                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex + 1);
            }

            // Do we have time to refine the path planning?
            if (edge.estimatedExecTime > config_.minEdgeTimeToRefinePath)
            {
                // Cancel curring path planner thread:

                // Launch a new planner from the new predicted pose:
            }
        }
        else
        {
            MRPT_LOG_INFO_STREAM(
                "Generating single motion cmd to move from node ID "
                << nCurr.nodeID_ << " => " << nNext.nodeID_
                << " CMD:" << generatedMotionCmd->asString());

            config_.vehicleMotionInterface->motion_execute(
                generatedMotionCmd, std::nullopt);

            _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex);
        }

        // new motion generated and sent out. We are done.
        return;
    }

    // If we are here, it is because we are in the middle of a navigation,
    // an edge motion is under execution and we are waiting for its end.
    // Send out a "dead man's switch" reset signal:
    config_.vehicleMotionInterface->motion_execute(std::nullopt, std::nullopt);
}

void WaypointSequencer::send_planner_output_to_viz(const PathPlannerOutput& ppo)
{
    ASSERT_(config_.vizSceneToModify);

    // Visualize the motion tree:
    // ----------------------------------
    RenderOptions ro;
    ro.highlight_path_to_node_id = ppo.po.bestNodeId;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    mrpt::opengl::CSetOfObjects::Ptr planViz =
        render_tree(ppo.po.motionTree, ppo.po.originalInput, ro);
    planViz->setName("astar_plan_result");

    planViz->setLocation(0, 0, 0.01);  // to easy the vis wrt the ground

    // Overlay the costmaps, if any:
    // ----------------------------------
    if (!ppo.costEvaluators.empty())
    {
        auto glCostMaps = mrpt::opengl::CSetOfObjects::Create();
        glCostMaps->setName("glCostMaps");

        float zOffset = 0.01f;  // to help visualize several costmaps at once

        for (const auto& ce : ppo.costEvaluators)
        {
            if (!ce) continue;
            auto glCostMap = ce->get_visualization();

            zOffset += 0.01f;
            glCostMap->setLocation(0, 0, zOffset);
            glCostMaps->insert(glCostMap);
        }

        planViz->insert(glCostMaps);
    }

    // Send to the viz "server":
    // ----------------------------------
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

void WaypointSequencer::send_path_to_viz(
    const MotionPrimitivesTreeSE2& tree, const TNodeID finalNode,
    const PlannerInput&                    originalPlanInput,
    const std::vector<CostEvaluator::Ptr>& costEvaluators)
{
    ASSERT_(config_.vizSceneToModify);

    // Visualize the motion tree:
    // ----------------------------------
    RenderOptions ro;
    ro.highlight_path_to_node_id = finalNode;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    mrpt::opengl::CSetOfObjects::Ptr planViz =
        render_tree(tree, originalPlanInput, ro);
    planViz->setName("astar_plan_result");

    planViz->setLocation(0, 0, 0.01);  // to easy the vis wrt the ground

    // Overlay the costmaps, if any:
    // ----------------------------------
    if (!costEvaluators.empty())
    {
        auto glCostMaps = mrpt::opengl::CSetOfObjects::Create();
        glCostMaps->setName("glCostMaps");

        float zOffset = 0.01f;  // to help visualize several costmaps at once

        for (const auto& ce : costEvaluators)
        {
            if (!ce) continue;
            auto glCostMap = ce->get_visualization();

            zOffset += 0.01f;
            glCostMap->setLocation(0, 0, zOffset);
            glCostMaps->insert(glCostMap);
        }

        planViz->insert(glCostMaps);
    }

    // Send to the viz "server":
    // ----------------------------------
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

void WaypointSequencer::send_current_state_to_viz()
{
    if (!config_.vizSceneToModify) return;

    const auto& _ = innerState_;

    auto glStateDetails = mrpt::opengl::CSetOfObjects::Create();
    glStateDetails->setName("glStateDetails");
    glStateDetails->setLocation(0, 0, 0.02);  // to easy the vis wrt the ground

    // last poses track:
    if (const auto& poses = _.latestPoses; !poses.empty())
    {
        auto glRobotPath = mrpt::opengl::CSetOfLines::Create();
        glRobotPath->setColor_u8(0x80, 0x80, 0x80, 0x80);
        const auto p0 = poses.begin()->second;
        glRobotPath->appendLine(p0.x, p0.y, 0, p0.x, p0.y, 0);
        for (const auto& p : poses)
        {
            glRobotPath->appendLineStrip(p.second.x, p.second.y, 0);

            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.1, 1.0);
            glCorner->setPose(p.second);
            glStateDetails->insert(glCorner);
        }
        glStateDetails->insert(glRobotPath);
    }

    if (const auto& actCond = _.activeEnqueuedConditionForViz;
        actCond.has_value())
    {
        const auto p   = actCond->position;
        const auto tol = actCond->tolerance;

        const mrpt::math::TPose2D p0 = {
            p.x - tol.x, p.y - tol.y, p.phi - tol.phi};
        const mrpt::math::TPose2D p1 = {
            p.x + tol.x, p.y + tol.y, p.phi + tol.phi};

        auto glCondPoly = mrpt::opengl::CSetOfLines::Create();
        glCondPoly->setColor_u8(0x40, 0x40, 0x40, 0xa0);

        glCondPoly->appendLine(p0.x, p0.y, 0, p1.x, p0.y, 0);
        glCondPoly->appendLineStrip(p1.x, p1.y, 0);
        glCondPoly->appendLineStrip(p0.x, p1.y, 0);
        glCondPoly->appendLineStrip(p0.x, p0.y, 0);

        glStateDetails->insert(glCondPoly);

        {
            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.15, 1.0);
            glCorner->setPose(p0);
            glStateDetails->insert(glCorner);
        }
        {
            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.15, 1.0);
            glCorner->setPose(p1);
            glStateDetails->insert(glCorner);
        }
    }

    if (const auto& predPose = _.collisionCheckingPosePrediction;
        predPose.has_value())
    {
        auto glVehShape = mrpt::opengl::CSetOfLines::Create();

        glVehShape->setLineWidth(1);
        glVehShape->setColor_u8(0x40, 0x40, 0x40, 0x80);

        render_vehicle(config_.ptgs.robotShape, *glVehShape);

        glVehShape->setPose(predPose.value());

        glStateDetails->insert(glVehShape);
    }

    // Send to the viz "server":
    // ----------------------------------
    // lock:
    if (config_.on_viz_pre_modify) config_.on_viz_pre_modify();

    if (auto glObj =
            config_.vizSceneToModify->getByName(glStateDetails->getName());
        glObj)
    {
        auto glContainer =
            std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(glObj);
        ASSERT_(glContainer);
        *glContainer = *glStateDetails;
    }
    else
    {
        config_.vizSceneToModify->insert(glStateDetails);
    }

    // unlock:
    if (config_.on_viz_post_modify) config_.on_viz_post_modify();
}
