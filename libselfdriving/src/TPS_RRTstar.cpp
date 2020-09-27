/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/TPS_RRTstar.h>
#include <libselfdriving/bestTrajectory.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <iostream>

using namespace selfdrive;

/* Algorithms (to be developed in future paper?)
 *
 * ================================================================
 *  TPS-RRT*
 * ================================================================
 * Q_T ← {q_0 }    # Tree nodes (configuration space)
 * E T ← ∅         # Tree edges
 * n ← 1           # Iteration counter
 *
 * while n ≤ N do
 *  q_rand ← SAMPLE( Q_free )
 *  Q_near ← NEAR_NODES( q_rand )
 *  q_best ← EXTEND( Q_near, q_rand )
 *  if q_rand \notin Q_T ∧ q_best != ∅ then
 *   Q_T ← Q_T U { q_rand }
 *   E_T ← E_T U { ( q_best , q_rand ) }
 *   n ← n + 1
 *   E_T ← REWIRE(Q_T, E_T, q_rand, Q_near )
 *  else if q_rand \in Q_T then
 *   q_prev ← PARENT( q_rand )
 *   if q_best != q_prev then
 *    E_T ← ( E_T \ { ( q_prev , q_rand ) } ) U {q_best, q_rand }
 *    E_T ← REWIRE ( Q_T , E_T , q_rand , Q_near )
 *
 * return (Q_T, E_T)
 *
 */

TPS_RRTstar::TPS_RRTstar() : mrpt::system::COutputLogger("TPS_RRTstar") {}

NavPlan TPS_RRTstar::plan(const PlannerInput& in)
{
    MRPT_START
    mrpt::system::CTimeLoggerEntry tleg(profiler_, "plan");

    // Sanity checks on inputs:
    ASSERT_(in.ptgs.initialized());
    ASSERT_NOT_EQUAL_(in.world_bbox_min, in.world_bbox_max);

    NavPlan ret;
    ret.original_input = in;

    // Calc maximum vehicle shape radius:
    double max_veh_radius = 0.;
    for (const auto& ptg : in.ptgs.ptgs)
        mrpt::keep_max(max_veh_radius, ptg->getMaxRobotRadius());

    // [Algo `tp_space_rrt`: Line 1]: Init tree adding the initial pose
    if (ret.move_tree.getAllNodes().empty())
    {
        result.move_tree.root = 0;
        result.move_tree.insertNode(
            result.move_tree.root, TNodeSE2_TP(pi.start_pose));
    }

    // Reuse MRPT's implementation:
    mrpt::nav::PlannerSimple2D grid_planner;

    ASSERT_ABOVE_(in.robot_shape.robot_radius, .0);
    grid_planner.robotRadius = static_cast<float>(in.robot_shape.robot_radius);
    grid_planner.minStepInReturnedPath = static_cast<float>(in.min_step_len);

    // Get obstacles:
    ASSERT_(in.obstacles);
    if (in.obstacles->dynamic())
        std::cerr << "[TPS_RRTstar] Warning: Ignoring dynamic obstacles.\n";

    profiler_.enter("plan.obstacles()");
    const auto obs_pts = in.obstacles->obstacles();
    profiler_.leave("plan.obstacles()");

    ASSERT_(obs_pts);
    float min_x, max_x, min_y, max_y, min_z, max_z;
    obs_pts->boundingBox(min_x, max_x, min_y, max_y, min_z, max_z);

    // make sure the bbox includes the goal & start poses as well as obstacles:
    mrpt::keep_max(max_x, in.state_start.pose.x + 1.0);
    mrpt::keep_max(max_x, in.state_goal.pose.x + 1.0);
    mrpt::keep_max(max_y, in.state_start.pose.y + 1.0);
    mrpt::keep_max(max_y, in.state_goal.pose.y + 1.0);

    mrpt::keep_min(min_x, in.state_start.pose.x - 1.0);
    mrpt::keep_min(min_x, in.state_goal.pose.x - 1.0);
    mrpt::keep_min(min_y, in.state_start.pose.y - 1.0);
    mrpt::keep_min(min_y, in.state_goal.pose.y - 1.0);

    // Build an occ. grid: all free except obstacles in the input
    // pointcloud.
    profiler_.enter("plan.build_grid");

    mrpt::maps::COccupancyGridMap2D obsGrid;
    obsGrid.setSize(
        min_x, max_x, min_y, max_y, static_cast<float>(params_.grid_resolution),
        0.99f);
    {
        const auto& xs = obs_pts->getPointsBufferRef_x();
        const auto& ys = obs_pts->getPointsBufferRef_y();

        for (std::size_t i = 0; i < xs.size(); i++)
            obsGrid.setPos(xs[i], ys[i], 0.01f /* low "freeness" */);
    }
    profiler_.leave("plan.build_grid");

    // Run A* itself:
    profiler_.enter("plan.computePath");

    std::deque<mrpt::math::TPoint2D> path;
    bool                             path_not_found;

    grid_planner.computePath(
        obsGrid, mrpt::poses::CPose2D(in.state_start.pose),
        mrpt::poses::CPose2D(in.state_goal.pose), path, path_not_found);

    profiler_.leave("plan.computePath");

    if (path_not_found || path.empty())
    {
        ret.success = false;
        return ret;
    }

    // Go thru the list of points and convert them into a sequence of PTG
    // actions:
    SE2_KinState last_state;
    last_state = in.state_start;

    // Make sure PTGs are initialized
    if (!in.ptgs.ptgs.empty())
    {
        mrpt::system::CTimeLoggerEntry tle(profiler_, "plan.init_PTGs");
        for (auto& ptg : in.ptgs.ptgs)
        {
            ASSERT_(ptg);
            ptg->initialize();
        }
    }

    for (const auto& p : path)
    {
        NavPlanAction act;
        act.state_from    = last_state;
        act.state_to.pose = mrpt::math::TPose2D(p.x, p.y, 0);

        // Compute PTG actions (trajectory segments):
        if (!in.ptgs.ptgs.empty())
        {
            mrpt::system::CTimeLoggerEntry tle(
                profiler_, "plan.bestTrajectory");

            // This finds the best PTG segments for the from/to poses.
            selfdrive::bestTrajectory(act, in.ptgs);
        }

        // for the next iter:
        // Note that "state_to" may have been modified by bestTrajectory().
        last_state = act.state_to;

        ret.actions.push_back(std::move(act));
    }

    ret.success = true;

    return ret;
    MRPT_END
}

// Auxiliary function:
void TPS_RRTstar::transformPointcloudWithSquareClipping(
    const mrpt::maps::CPointsMap& in_map, mrpt::maps::CPointsMap& out_map,
    const mrpt::poses::CPose2D& asSeenFrom, const double MAX_DIST_XY)
{
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    in_map.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    out_map.clear();
    out_map.reserve(nObs);  // Prealloc mem for speed-up

    const mrpt::poses::CPose2D invPose = -asSeenFrom;
    // We can safely discard the rest of obstacles, since they cannot be
    // converted into TP-Obstacles anyway!

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const double gx = obs_xs[obs], gy = obs_ys[obs];

        if (std::abs(gx - asSeenFrom.x()) > MAX_DIST_XY ||
            std::abs(gy - asSeenFrom.y()) > MAX_DIST_XY)
        {
            // ignore this obstacle: anyway, I don't know how to map it to
            // TP-Obs!
            continue;
        }

        double ox, oy;
        invPose.composePoint(gx, gy, ox, oy);

        out_map.insertPointFast(ox, oy, 0);
    }
}

void TPS_RRTstar::spaceTransformer(
    const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
    const double MAX_DIST, std::vector<double>& out_TPObstacles)
{
    MRPT_START

    // Take "k_rand"s and "distances" such that the collision hits the
    // obstacles
    // in the "grid" of the given PT
    // --------------------------------------------------------------------
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    // = in_obstacles.getPointsCount();
    in_obstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    // Init obs ranges:
    ptg.initTPObstacles(out_TPObstacles);

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const float ox = obs_xs[obs];
        const float oy = obs_ys[obs];

        if (std::abs(ox) > MAX_DIST || std::abs(oy) > MAX_DIST)
            continue;  // ignore this obstacle: anyway, I don't know how to
        // map it to TP-Obs!

        ptg.updateTPObstacle(ox, oy, out_TPObstacles);
    }

    // Leave distances in out_TPObstacles un-normalized ([0,1]), so they
    // just represent real distances in meters.
    MRPT_END
}

void TPS_RRTstar::spaceTransformerOneDirectionOnly(
    const int                           tp_space_k_direction,
    const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
    const double MAX_DIST, double& out_TPObstacle_k)
{
    MRPT_START
    // Take "k_rand"s and "distances" such that the collision hits the
    // obstacles
    // in the "grid" of the given PT
    // --------------------------------------------------------------------
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    // = in_obstacles.getPointsCount();
    in_obstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    // Init obs ranges:
    ptg.initTPObstacleSingle(tp_space_k_direction, out_TPObstacle_k);

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const float ox = obs_xs[obs];
        const float oy = obs_ys[obs];

        if (std::abs(ox) > MAX_DIST || std::abs(oy) > MAX_DIST)
            continue;  // ignore this obstacle: anyway, I don't know how to
        // map it to TP-Obs!

        ptg.updateTPObstacleSingle(
            ox, oy, tp_space_k_direction, out_TPObstacle_k);
    }

    // Leave distances in out_TPObstacles un-normalized ([0,1]), so they
    // just represent real distances in meters.

    MRPT_END
}
