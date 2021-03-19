/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <selfdriving/TPS_RRTstar.h>
#include <selfdriving/bestTrajectory.h>

#include <iostream>

using namespace selfdriving;

// clang-format off
/* Algorithm:
 *
 * ================================================================
 *  TPS-RRT*
 * ================================================================
 *  1  |  X_T ← {X_0 }    # Tree nodes (state space)
 *  2  |  E T ← ∅         # Tree edges
 *  3  | 
 *  4  |  for i \in [1,N] do
 *  5  |   q_i ← SAMPLE( Q_free )
 *  6  |   {x_best, x_i} ← argmin{x ∈ Tree | cost[x, q_i ] < r ∧ CollisionFree(pi(x,q_i)}( cost[x] + cost[x,x_i] )
 *  7  |   parent[x_i] ← x_best
 *  8  |   cost[x_i] ← cost[x_best] + cost[x_best, x_i]
 *  9  |  
 * 10  |   for all {x ∈ Tree ∪ {x goal } | cost[x, x_i ] < r ∧ cost[x_i] +cost[x_i,x]<cost[x] ∧ CollisionFree(pi(x,x_i)} do
 * 11  |    cost[x] ← cost[x_i] + cost[x_i, x]
 * 12  |    parent[x] ← x_i
 * 13  |   
 * 14  |   X_T ← X_T U { x_i }
 * 15  |   E_T ← E_T U { ( x_best, x_i ) }
 * 16  | 
 * 17  |  return (X_T, E_T)
 *
 */
// clang-format on

TPS_RRTstar::TPS_RRTstar() : mrpt::system::COutputLogger("TPS_RRTstar") {}

PlannerOutput TPS_RRTstar::plan(const PlannerInput& in)
{
    MRPT_START
    mrpt::system::CTimeLoggerEntry tleg(profiler_, "plan");

    // Sanity checks on inputs:
    ASSERT_(in.ptgs.initialized());
    ASSERT_(in.world_bbox_min != in.world_bbox_max);

    PlannerOutput po;
    po.originalInput = in;

#if 0
    // [Algo `tp_space_rrtstar`: Line 1]: Init tree adding the initial pose
    if (po.move_tree.getAllNodes().empty())
    {
        po.move_tree.root = 0;
        po.move_tree.insertNode(po.move_tree.root, TNodeSE2_TP(pi.start_pose));
    }
#endif

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
    mrpt::math::TBoundingBoxf bbox = obs_pts->boundingBox();

    // make sure the bbox includes the goal & start poses as well as obstacles:
    bbox.updateWithPoint(mrpt::math::TPoint3Df(
        in.state_start.pose.x + 1.0, in.state_start.pose.y + 1.0, .0f));

    bbox.updateWithPoint(mrpt::math::TPoint3Df(
        in.state_start.pose.x - 1.0, in.state_start.pose.y - 1.0, .0f));

    // Build an occ. grid: all free except obstacles in the input
    // pointcloud.
    profiler_.enter("plan.build_grid");

    mrpt::maps::COccupancyGridMap2D obsGrid;
    obsGrid.setSize(
        bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y,
        static_cast<float>(params_.grid_resolution), 0.99f);
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
        po.success = false;
        return po;
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

#if 0
    for (const auto& p : path)
    {
        NavPlanAction act;
        act.stateFrom    = last_state;
        act.stateTo.pose = mrpt::math::TPose2D(p.x, p.y, 0);

        // Compute PTG actions (trajectory segments):
        if (!in.ptgs.ptgs.empty())
        {
            mrpt::system::CTimeLoggerEntry tle(
                profiler_, "plan.bestTrajectory");

            // This finds the best PTG segments for the from/to poses.
            selfdriving::bestTrajectory(act, in.ptgs);
        }

        // for the next iter:
        // Note that "stateTo" may have been modified by bestTrajectory().
        last_state = act.stateTo;

        ret.actions.push_back(std::move(act));
    }

    ret.success = true;
#endif

    return po;
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
