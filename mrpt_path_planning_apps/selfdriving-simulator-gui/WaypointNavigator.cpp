/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include "WaypointNavigator.h"

#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/render_tree.h>
#include <mpp/algos/trajectories.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSphere.h>

#include <chrono>

using namespace mpp;

namespace
{
const char* VIZ_REFERENCE_PATH = "nav_reference_path";
const char* VIZ_FOLLOWER_STATE = "nav_follower_state";
const char* VIZ_SINGLE_PLAN    = "astar_plan_result";

// Planning uses the PTG objects, shared by all copies of the PTG config:
std::mutex planMtx;

const char* to_string(FollowerStatus s)
{
    switch (s)
    {
        case FollowerStatus::Idle:
            return "Idle";
        case FollowerStatus::Running:
            return "Running";
        case FollowerStatus::ReachedGoal:
            return "ReachedGoal";
        case FollowerStatus::Blocked:
            return "Blocked";
        case FollowerStatus::OffPathExceeded:
            return "OffPathExceeded";
        case FollowerStatus::MissedGoal:
            return "MissedGoal";
    };
    return "?";
}
}  // namespace

WaypointNavigator::WaypointNavigator()
    : mrpt::system::COutputLogger("WaypointNavigator")
{
}

WaypointNavigator::~WaypointNavigator()
{
    closing_ = true;
    if (plannerThread_.joinable()) { plannerThread_.join(); }
    if (controlThread_.joinable()) { controlThread_.join(); }
}

void WaypointNavigator::set_static_obstacles(
    const mrpt::maps::CPointsMap::Ptr& pts)
{
    auto lck         = mrpt::lockHelper(mtx_);
    staticObstacles_ = pts;
}

void WaypointNavigator::start()
{
    ASSERT_(config.vehicle);
    ASSERT_(!controlThread_.joinable());

    follower.setRobotShape(config.ptgs.robotShape);
    if (config.ptgs.minTurningRadius > 0)
    {
        follower.params.min_turn_radius = config.ptgs.minTurningRadius;
    }

    controlThread_ = std::thread(&WaypointNavigator::control_loop, this);
}

void WaypointNavigator::set_status(const std::string& s)
{
    auto lck = mrpt::lockHelper(mtx_);
    // Only log changes of state, not of the progress text after ':'
    if (s.substr(0, s.find(':')) != status_.substr(0, status_.find(':')))
    {
        MRPT_LOG_INFO_STREAM("Status: " << s);
    }
    status_ = s;
}

std::string WaypointNavigator::status_text() const
{
    auto lck = mrpt::lockHelper(mtx_);
    return status_;
}

std::optional<uint64_t> WaypointNavigator::begin_planning()
{
    if (planning_)
    {
        MRPT_LOG_WARN("Ignoring request: still planning a previous one.");
        return std::nullopt;
    }
    if (plannerThread_.joinable()) { plannerThread_.join(); }
    planning_ = true;
    set_status("Planning...");

    auto lck = mrpt::lockHelper(mtx_);
    return ++requestGen_;
}

bool WaypointNavigator::is_current_request(uint64_t gen) const
{
    auto lck = mrpt::lockHelper(mtx_);
    return gen == requestGen_ && !closing_;
}

void WaypointNavigator::request_navigation(const WaypointSequence& wps)
{
    if (wps.waypoints.empty())
    {
        MRPT_LOG_WARN("Ignoring navigation request: no waypoints.");
        return;
    }
    const auto gen = begin_planning();
    if (!gen) { return; }

    plannerThread_ = std::thread(
        [this, wps, gen = *gen]()
        {
            try
            {
                const auto loc = config.vehicle->get_localization();
                ASSERTMSG_(loc.valid, "Could not get the vehicle pose");

                SE2_KinState start;
                start.pose = loc.pose;

                Trajectory fullPath;
                for (size_t i = 0;
                     i < wps.waypoints.size() && is_current_request(gen); i++)
                {
                    const auto& wp = wps.waypoints[i];

                    SE2orR2_KinState goal;
                    if (wp.targetHeading.has_value())
                    {
                        goal.state = PoseOrPoint(wp.targetAsPose());
                    }
                    else { goal.state = PoseOrPoint(wp.target); }

                    set_status(mrpt::format(
                        "Planning to waypoint %zu/%zu...", i + 1,
                        wps.waypoints.size()));

                    const auto leg = plan(start, goal);
                    if (!leg.success)
                    {
                        set_status(mrpt::format(
                            "Planning failed to waypoint %zu", i + 1));
                        planning_ = false;
                        return;
                    }

                    // <=0 means: the follower max speed
                    const double speed =
                        wp.speedRatio < 1.0
                            ? wp.speedRatio * follower.params.max_speed
                            : 0.0;
                    const auto legPath = plan_to_reference_path(leg, speed);

                    // Each leg starts where the previous one ended:
                    const size_t first = fullPath.empty() ? 0 : 1;
                    for (size_t k = first; k < legPath.size(); k++)
                    {
                        fullPath.push_back(legPath[k]);
                    }
                    start      = SE2_KinState();
                    start.pose = fullPath.back().pose;
                }
                set_reference_path(fullPath, gen);
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM("Planning error: " << e.what());
                set_status("Planning error");
            }
            planning_ = false;
        });
}

void WaypointNavigator::request_single_plan(
    const SE2_KinState& start, const SE2orR2_KinState& goal)
{
    const auto gen = begin_planning();
    if (!gen) { return; }

    plannerThread_ = std::thread(
        [this, start, goal, gen = *gen]()
        {
            try
            {
                const auto out = plan_single(start, goal);

                // Converted here, so following it later does not need the
                // PTGs while another plan may be running:
                Trajectory path;
                if (out.success) { path = plan_to_reference_path(out, 0.0); }
                if (is_current_request(gen))
                {
                    {
                        auto lck        = mrpt::lockHelper(mtx_);
                        lastSinglePath_ = std::move(path);
                    }
                    set_status(out.success ? "Plan ready" : "Planning failed");
                }
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM("Planning error: " << e.what());
                set_status("Planning error");
            }
            planning_ = false;
        });
}

void WaypointNavigator::follow_last_plan()
{
    Trajectory path;
    {
        auto lck = mrpt::lockHelper(mtx_);
        path     = lastSinglePath_;
    }
    if (path.empty())
    {
        MRPT_LOG_WARN("There is no successful plan to follow.");
        return;
    }
    set_reference_path(path);
}

void WaypointNavigator::suspend()
{
    suspended_ = true;
    set_status("Suspended");
}

void WaypointNavigator::resume()
{
    suspended_ = false;
    set_status("Navigating");
}

void WaypointNavigator::cancel()
{
    {
        auto lck = mrpt::lockHelper(mtx_);
        follower.reset();
        suspended_ = false;
        requestGen_++;  // discards any plan being computed
    }
    set_status("Canceled");
}

bool WaypointNavigator::set_reference_path(
    const Trajectory& path, std::optional<uint64_t> gen)
{
    {
        auto lck = mrpt::lockHelper(mtx_);
        if (gen && (*gen != requestGen_ || closing_)) { return false; }
        follower.setTrajectory(path);
        suspended_ = false;
        status_    = "Navigating";
    }
    MRPT_LOG_INFO_STREAM(
        "New reference path: " << path.size() << " points, "
                               << follower.totalLength() << " m");
    viz_reference_path(path);
    return true;
}

PlannerOutput WaypointNavigator::plan(
    const SE2_KinState& start, const SE2orR2_KinState& goal,
    std::vector<CostEvaluator::Ptr>* costEvaluatorsOut)
{
    mrpt::maps::CPointsMap::Ptr obs;
    {
        auto lck = mrpt::lockHelper(mtx_);
        obs      = staticObstacles_;
    }
    if (!obs) { obs = mrpt::maps::CSimplePointsMap::Create(); }

    PlannerInput pi;
    pi.stateStart = start;
    pi.stateGoal  = goal;
    pi.ptgs       = config.ptgs;
    pi.obstacles.push_back(ObstacleSource::FromStaticPointcloud(obs));

    // Planning area: the obstacles, plus some margin around start and goal.
    const double margin   = config.plannerBboxMargin;
    const auto   ptStart  = mrpt::math::TPoint3D(start.pose.x, start.pose.y, 0);
    const auto   goalPose = goal.asSE2KinState().pose;
    const auto   ptGoal   = mrpt::math::TPoint3D(goalPose.x, goalPose.y, 0);
    const auto   d        = mrpt::math::TPoint3D(margin, margin, 0);

    auto bbox = mrpt::math::TBoundingBox(ptStart - d, ptStart + d);
    bbox.updateWithPoint(ptGoal - d);
    bbox.updateWithPoint(ptGoal + d);
    if (!obs->empty())
    {
        const auto ob = obs->boundingBox();
        bbox.updateWithPoint(ob.min.cast<double>());
        bbox.updateWithPoint(ob.max.cast<double>());
    }
    pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};
    pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};

    TPS_Astar planner;
    planner.params_ = config.plannerParams;
    planner.setMinLoggingLevel(getMinLoggingLevel());

    if (config.globalCostParams && !obs->empty())
    {
        planner.costEvaluators_.push_back(
            CostEvaluatorCostMap::FromStaticPointObstacles(
                *obs, *config.globalCostParams, start.pose,
                config.ptgs.robotShape));
    }

    MRPT_LOG_INFO_STREAM(
        "Planning from " << start.pose.asString() << " to " << goal.asString());

    auto lck = mrpt::lockHelper(planMtx);
    auto out = planner.plan(pi);

    MRPT_LOG_INFO_STREAM(
        "Planning " << (out.success ? "succeeded" : "FAILED") << " in "
                    << out.computationTime
                    << " s, path cost: " << out.pathCost);

    if (costEvaluatorsOut) { *costEvaluatorsOut = planner.costEvaluators_; }
    return out;
}

Trajectory WaypointNavigator::plan_to_reference_path(
    const PlannerOutput& plan, double targetSpeed) const
{
    ASSERT_(plan.success);
    ASSERT_(plan.bestNodeId.has_value());

    // Uses the PTGs, shared with any plan running meanwhile:
    auto lck = mrpt::lockHelper(planMtx);

    auto [path, edges] = plan.motionTree.backtrack_path(*plan.bestNodeId);

    // Make PTG edges connect the exact node poses:
    refine_trajectory(path, edges, plan.originalInput.ptgs);

    const auto samples = plan_to_trajectory(
        edges, plan.originalInput.ptgs, 0.1 /*sample period [s]*/);

    // Samples are relative to the start pose:
    const auto startPose =
        mrpt::poses::CPose2D(plan.originalInput.stateStart.pose);

    Trajectory ref;
    ref.reserve(samples.size());
    for (const auto& [t, s] : samples)
    {
        const auto p = startPose + mrpt::poses::CPose2D(s.state.pose);
        ref.emplace_back(p.asTPose(), targetSpeed);
    }
    return ref;
}

PlannerOutput WaypointNavigator::plan_single(
    const SE2_KinState& start, const SE2orR2_KinState& goal)
{
    std::vector<CostEvaluator::Ptr> costEvaluators;
    const auto                      out = plan(start, goal, &costEvaluators);

    // Search tree, with the best path highlighted:
    RenderOptions ro;
    ro.highlight_path_to_node_id = out.bestNodeId;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    auto glPlan = render_tree(out.motionTree, out.originalInput, ro);
    glPlan->setName(VIZ_SINGLE_PLAN);
    glPlan->setLocation(0, 0, 0.01);

    for (const auto& ce : costEvaluators)
    {
        if (ce) { glPlan->insert(ce->get_visualization()); }
    }
    viz_replace(glPlan);

    return out;
}

void WaypointNavigator::control_loop()
{
    using clock = std::chrono::steady_clock;

    auto next = clock::now();
    while (!closing_)
    {
        try
        {
            control_step();
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_THROTTLE_ERROR_STREAM(5.0, "Control loop: " << e.what());
        }
        next += std::chrono::microseconds(
            static_cast<int64_t>(1e6 * follower.params.control_period));
        std::this_thread::sleep_until(next);
    }

    if (driving_) { config.vehicle->stop(StopKind::REGULAR); }
}

void WaypointNavigator::control_step()
{
    auto& veh = *config.vehicle;

    {
        auto lck = mrpt::lockHelper(mtx_);
        if (!follower.hasTrajectory() || suspended_)
        {
            lck.unlock();
            if (driving_)
            {
                veh.stop(StopKind::REGULAR);
                driving_ = false;
            }
            return;
        }
    }

    const auto loc = veh.get_localization();
    if (!loc.valid)
    {
        MRPT_LOG_THROTTLE_WARN(2.0, "No valid localization: stopping.");
        veh.stop(StopKind::EMERGENCY);
        return;
    }
    const auto odo = veh.get_odometry();

    // Live obstacles for the predictive safety layer:
    std::optional<mrpt::maps::CSimplePointsMap> livePts;
    if (config.lidar)
    {
        if (const auto scan = config.lidar->last_lidar_obs();
            scan && scan->timestamp != lastScanStamp_)
        {
            lastScanStamp_ = scan->timestamp;
            livePts.emplace();
            livePts->insertObservation(*scan, mrpt::poses::CPose3D(loc.pose));
        }
    }

    TrajectoryFollower::Output out;
    double                     totalLength = 0;
    {
        auto lck = mrpt::lockHelper(mtx_);
        if (!follower.hasTrajectory())
        {
            return;  // canceled meanwhile
        }
        if (livePts) { follower.setObstacles(*livePts); }
        out         = follower.step(loc, odo);
        totalLength = follower.totalLength();
    }

    switch (out.status)
    {
        case FollowerStatus::ReachedGoal:
        case FollowerStatus::OffPathExceeded:
        case FollowerStatus::MissedGoal:
        {
            veh.stop(StopKind::REGULAR);
            driving_ = false;
            {
                auto lck = mrpt::lockHelper(mtx_);
                follower.reset();
            }
            set_status(
                out.status == FollowerStatus::ReachedGoal
                    ? std::string("Goal reached")
                    : mrpt::format("Failed: %s", to_string(out.status)));
            break;
        }
        case FollowerStatus::Blocked:
            // Wait, stopped, for the way to clear:
            veh.stop(StopKind::REGULAR);
            driving_ = false;
            set_status("Blocked by obstacles");
            break;

        default:
            veh.follow(out.command);
            driving_ = true;
            set_status(mrpt::format(
                "Navigating: %.1f/%.1f m, v=%.2f m/s", out.arc_length_s,
                totalLength, out.target_speed));
            break;
    };

    viz_follower_state(out);
}

void WaypointNavigator::viz_replace(const mrpt::viz::CSetOfObjects::Ptr& obj)
{
    if (!config.vizScene) { return; }
    if (config.on_viz_pre_modify) { config.on_viz_pre_modify(); }

    if (auto prev = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(
            config.vizScene->getByName(obj->getName()));
        prev)
    {
        *prev = *obj;
    }
    else { config.vizScene->insert(obj); }

    if (config.on_viz_post_modify) { config.on_viz_post_modify(); }
}

void WaypointNavigator::viz_reference_path(const Trajectory& path)
{
    auto glPath = mrpt::viz::CSetOfObjects::Create();
    glPath->setName(VIZ_REFERENCE_PATH);

    auto lines = mrpt::viz::CSetOfLines::Create();
    lines->setColor_u8(0x00, 0x60, 0xff);
    lines->setLineWidth(3.0f);
    for (size_t i = 1; i < path.size(); i++)
    {
        const auto& a = path[i - 1].pose;
        const auto& b = path[i].pose;
        lines->appendLine(a.x, a.y, 0.03, b.x, b.y, 0.03);
    }
    glPath->insert(lines);

    viz_replace(glPath);
}

void WaypointNavigator::viz_follower_state(
    const TrajectoryFollower::Output& out)
{
    auto glState = mrpt::viz::CSetOfObjects::Create();
    glState->setName(VIZ_FOLLOWER_STATE);

    // The commanded chunk (its odom frame is the map frame in the simulator):
    auto lines = mrpt::viz::CSetOfLines::Create();
    lines->setColor_u8(0x00, 0xc0, 0x00);
    lines->setLineWidth(4.0f);
    const auto& pts = out.command.points;
    for (size_t i = 1; i < pts.size(); i++)
    {
        const auto& a = pts[i - 1].pose;
        const auto& b = pts[i].pose;
        lines->appendLine(a.x, a.y, 0.05, b.x, b.y, 0.05);
    }
    glState->insert(lines);

    auto lookahead = mrpt::viz::CSphere::Create(0.06f);
    lookahead->setColor_u8(0xff, 0x80, 0x00);
    lookahead->setLocation(out.lookahead_point.x, out.lookahead_point.y, 0.05);
    glState->insert(lookahead);

    viz_replace(glState);
}
