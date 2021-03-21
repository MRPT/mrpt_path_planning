/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
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
 *     | 
 *  3  |  for i \in [1,N] do
 *  4  |   q_i ← SAMPLE( Q_free )
 *  5  |   {x_best, x_i} ← argmin{x ∈ Tree | cost[x, q_i ] < r ∧ CollisionFree(pi(x,q_i)}( cost[x] + cost[x,x_i] )
 *  6  |   parent[x_i] ← x_best
 *  7  |   cost[x_i] ← cost[x_best] + cost[x_best, x_i]
 *     |  
 *  8  |   for all {x ∈ Tree ∪ {x goal } | cost[x, x_i ] < r ∧ cost[x_i] +cost[x_i,x]<cost[x] ∧ CollisionFree(pi(x,x_i)} do
 *  9  |    cost[x] ← cost[x_i] + cost[x_i, x]
 * 10  |    parent[x] ← x_i
 *     |   
 * 11  |   X_T ← X_T U { x_i }
 * 12  |   E_T ← E_T U { ( x_best, x_i ) }
 *     | 
 * 13  |  return (X_T, E_T)
 *
 */
// clang-format on

TPS_RRTstar::TPS_RRTstar() : mrpt::system::COutputLogger("TPS_RRTstar") {}

static bool within_bbox(
    const mrpt::math::TPose2D& p, const mrpt::math::TPose2D& max,
    const mrpt::math::TPose2D& min)
{
    return p.x < max.x && p.y < max.y && p.phi < max.phi &&  //
           p.x > min.x && p.y > min.y && p.phi > min.phi;
}

PlannerOutput TPS_RRTstar::plan(const PlannerInput& in)
{
    MRPT_START
    mrpt::system::CTimeLoggerEntry tleg(profiler_, "plan");

    // Sanity checks on inputs:
    ASSERT_(in.ptgs.initialized());
    ASSERT_(in.worldBboxMin != in.worldBboxMax);
    ASSERT_(within_bbox(in.stateStart.pose, in.worldBboxMax, in.worldBboxMin));
    ASSERT_(within_bbox(in.stateGoal.pose, in.worldBboxMax, in.worldBboxMin));

    PlannerOutput po;
    po.originalInput = in;

    auto& tree = po.motionTree;  // shortcut

    //  1  |  X_T ← {X_0 }    # Tree nodes (state space)
    // ------------------------------------------------------------------
    tree.root = tree.next_free_node_ID();
    tree.insert_node(tree.root, in.stateStart);

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    // Prepare draw params:
    DrawFreePoseParams drawParams(in);

    //  3  |  for i \in [1,N] do
    for (size_t rrtIter = 0; rrtIter < in.maxPlanIterations; rrtIter++)
    {
        // 4  |   q_i ← SAMPLE( Q_free )
        // ------------------------------------------------------------------
        // issue: What about dynamic obstacles that depend on time?

        const mrpt::math::TPose2D qi = draw_random_free_pose(drawParams);

        //  5  |   {x_best, x_i} ← argmin{x ∈ Tree | cost[x, q_i ] < r ∧
        //  CollisionFree(pi(x,q_i)}( cost[x] + cost[x,x_i] )
        // ------------------------------------------------------------------

        //  6  |   parent[x_i] ← x_best
        // ------------------------------------------------------------------

        //  7  |   cost[x_i] ← cost[x_best] + cost[x_best, x_i]
        // ------------------------------------------------------------------

        //  8  |   for all {x ∈ Tree ∪ {x goal } | cost[x, x_i ] < r ∧ cost[x_i]
        //  +cost[x_i,x]<cost[x] ∧ CollisionFree(pi(x,x_i)} do
        // ------------------------------------------------------------------

        //  9  |    cost[x] ← cost[x_i] + cost[x_i, x]
        // ------------------------------------------------------------------

        // 10  |    parent[x] ← x_i
        // ------------------------------------------------------------------

        // 11  |   X_T ← X_T U { x_i }
        // ------------------------------------------------------------------

        // 12  |   E_T ← E_T U { ( x_best, x_i ) }
        // ------------------------------------------------------------------

    }  // for each rrtIter

    // ----------

    if (0)
    {
        po.success = false;
        return po;
    }

    // Go thru the list of points and convert them into a sequence of PTG
    // actions:
    SE2_KinState last_state;
    last_state = in.stateStart;

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
void TPS_RRTstar::transform_pc_square_clipping(
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

std::vector<double> TPS_RRTstar::transform_to_tps(
    const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
    const double MAX_DIST)
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
    std::vector<double> out_TPObstacles;
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
    return out_TPObstacles;
    MRPT_END
}

double TPS_RRTstar::transform_to_tps_single_path(
    const int                           tp_space_k_direction,
    const mrpt::maps::CSimplePointsMap& in_obstacles, const ptg_t& ptg,
    const double MAX_DIST)
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
    double out_TPObstacle_k = 0;
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
    MRPT_TODO("check normalization?");
    return out_TPObstacle_k;

    MRPT_END
}

mrpt::math::TPose2D TPS_RRTstar::draw_random_free_pose(
    const TPS_RRTstar::DrawFreePoseParams& p)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "draw_random_free_pose");

    auto& rng = mrpt::random::getRandomGenerator();

    const auto obstacles = p.pi_.obstacles->obstacles();

    // P[Select goal] = goalBias
    if (rng.drawUniform(0, 1) < params_.goalBias)
    {
        // goal selected:
        return p.pi_.stateGoal.pose;
    }
    else
    {
        // Pick a random pose until we find a collision-free one:
        const auto& bbMin = p.pi_.worldBboxMin;
        const auto& bbMax = p.pi_.worldBboxMax;

        const size_t maxAttempts = 1000000;

        for (size_t attempt = 0; attempt < maxAttempts; attempt++)
        {
            // tentative pose:
            const auto q = mrpt::math::TPose2D(
                rng.drawUniform(bbMin.x, bbMax.x),
                rng.drawUniform(bbMin.y, bbMax.y),
                rng.drawUniform(bbMin.phi, bbMax.phi));

            // TODO: More flexible check? Variable no. of points?
            mrpt::math::TPoint2D closestObs;
            float                closestDistSqr;
            obstacles->kdTreeClosestPoint2D(
                {q.x, q.y}, closestObs, closestDistSqr);

            const auto closestObsWrtRobot = q.inverseComposePoint(closestObs);

            const bool isCollision = selfdriving::obstaclePointCollides(
                closestObsWrtRobot, p.pi_.ptgs);

            if (!isCollision) return q;
        }
        THROW_EXCEPTION("Could not draw collision-free random pose!");
    }

    return {};  // should never reach here
}

std::set<TNodeID> TPS_RRTstar::find_nodes_within_ball(
    const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
    const double maxDistance)
{
    ASSERT_(!nodes_.empty());
    double min_d  = std::numeric_limits<double>::max();
    auto   min_id = INVALID_NODEID;
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it)
    {
        if (ignored_nodes &&
            ignored_nodes->find(it->first) != ignored_nodes->end())
            continue;  // ignore it
        const NODE_TYPE_FOR_METRIC ptTo(query_pt.state);
        const NODE_TYPE_FOR_METRIC ptFrom(it->second.state);
        // Skip the more expensive calculation of exact distance:
        if (distanceMetricEvaluator.cannotBeNearerThan(ptFrom, ptTo, min_d))
            continue;
        double d = distanceMetricEvaluator.distance(ptFrom, ptTo);
        if (d < min_d)
        {
            min_d  = d;
            min_id = it->first;
        }
    }
    if (out_distance) *out_distance = min_d;
    return min_id;
}
