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

    // clipping dist for all ptgs:
    MRPT_TODO("go over all ptgs");
    const double MAX_XY_DIST = in.ptgs.ptgs.at(0)->getRefDistance();

    //  1  |  X_T ← {X_0 }    # Tree nodes (state space)
    // ------------------------------------------------------------------
    tree.root = tree.next_free_node_ID();
    tree.insert_node(tree.root, in.stateStart);

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    // Prepare draw params:
    DrawFreePoseParams drawParams(in, tree);

    double searchRadius = params_.initialSearchRadius;

    //  3  |  for i \in [1,N] do
    for (size_t rrtIter = 0; rrtIter < params_.maxIterations; rrtIter++)
    {
        // 4  |   q_i ← SAMPLE( Q_free )
        // ------------------------------------------------------------------
        // issue: What about dynamic obstacles that depend on time?
        const mrpt::math::TPose2D qi = draw_random_free_pose(drawParams);

        //  5  |   {x_best, x_i} ← argmin{x ∈ Tree | cost[x, q_i ] < r ∧
        //  CollisionFree(pi(x,q_i)}( cost[x] + cost[x,x_i] )
        // ------------------------------------------------------------------
        const closest_nodes_list_t closeNodes =
            find_nodes_within_ball(tree, qi, searchRadius, in.ptgs);

        // No body around?
        if (closeNodes.empty()) continue;

        MRPT_LOG_DEBUG_STREAM(
            "iter: " << rrtIter << ", " << closeNodes.size()
                     << " candidate nodes near qi=" << qi.asString());

        // Check for CollisionFree and keep the smallest cost:
        cached_local_obstacles(tree, nodeId, obstacles, MAX_XY_DIST);

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

void TPS_RRTstar::transform_pc_square_clipping(
    const mrpt::maps::CPointsMap& inMap, const mrpt::poses::CPose2D& asSeenFrom,
    const double MAX_DIST_XY, mrpt::maps::CPointsMap& outMap)
{
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    inMap.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    outMap.clear();
    outMap.reserve(nObs);  // Prealloc mem for speed-up

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

        outMap.insertPointFast(ox, oy, 0);
    }
}

distance_t TPS_RRTstar::tp_obstacles_single_path(
    const trajectory_index_t      tp_space_k_direction,
    const mrpt::maps::CPointsMap& localObstacles, const ptg_t& ptg)
{
    MRPT_START
    // Take "k_rand"s and "distances" such that the collision hits the
    // obstacles
    // in the "grid" of the given PT
    // --------------------------------------------------------------------
    size_t       nObs;
    const float *obs_xs, *obs_ys, *obs_zs;
    localObstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

    // Init obs ranges:
    normalized_distance_t out_TPObstacle_k = 0;
    ptg.initTPObstacleSingle(tp_space_k_direction, out_TPObstacle_k);

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const float ox = obs_xs[obs];
        const float oy = obs_ys[obs];

        ptg.updateTPObstacleSingle(
            ox, oy, tp_space_k_direction, out_TPObstacle_k);
    }

    // Leave distances in out_TPObstacles un-normalized, so they
    // just represent real distances in "pseudo-meters".
    return out_TPObstacle_k;

    MRPT_END
}

mrpt::math::TPose2D TPS_RRTstar::draw_random_free_pose(
    const TPS_RRTstar::DrawFreePoseParams& p)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "draw_random_free_pose");

    auto& rng = mrpt::random::getRandomGenerator();

    // P[Select goal] = goalBias
    if (rng.drawUniform(0, 1) < params_.goalBias)
    {
        // goal selected:
        return p.pi_.stateGoal.pose;
    }
    else
    {
        if (params_.drawInTPS)
            return draw_random_tps(p);
        else
            return draw_random_euclidean(p);
    }
}

mrpt::math::TPose2D TPS_RRTstar::draw_random_euclidean(
    const TPS_RRTstar::DrawFreePoseParams& p)
{
    auto tle = mrpt::system::CTimeLoggerEntry(
        profiler_, "draw_random_free_pose.euclidean");

    auto& rng = mrpt::random::getRandomGenerator();

    const auto obstacles = p.pi_.obstacles->obstacles();

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
        obstacles->kdTreeClosestPoint2D({q.x, q.y}, closestObs, closestDistSqr);

        const auto closestObsWrtRobot = q.inverseComposePoint(closestObs);

        const bool isCollision =
            selfdriving::obstaclePointCollides(closestObsWrtRobot, p.pi_.ptgs);

        if (!isCollision) return q;
    }
    THROW_EXCEPTION("Could not draw collision-free random pose!");
}

mrpt::math::TPose2D TPS_RRTstar::draw_random_tps(
    const TPS_RRTstar::DrawFreePoseParams& p)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "draw_random_free_pose.tps");

    auto& rng = mrpt::random::getRandomGenerator();

    const auto obstacles = p.pi_.obstacles->obstacles();

    const size_t maxAttempts = 1000000;
    for (size_t attempt = 0; attempt < maxAttempts; attempt++)
    {
        // draw source node, then ptg index, then trajectory index, then
        // distance:
        const auto  nodeIdx = rng.drawUniform32bit() % p.tree_.nodes().size();
        const auto& node    = p.tree_.nodes().at(nodeIdx);

        const auto  ptgIdx = rng.drawUniform32bit() % p.pi_.ptgs.ptgs.size();
        const auto& ptg    = p.pi_.ptgs.ptgs.at(ptgIdx);

        const auto trajIdx =
            rng.drawUniform32bit() % ptg->getAlphaValuesCount();
        const auto trajDist =
            rng.drawUniform(params_.minStepLength, params_.maxStepLength);

        // Let the PTG know about the current local velocity:
        ptg_t::TNavDynamicState ds;
        (ds.curVelLocal = node.vel).rotate(-node.pose.phi);
        ds.relTarget      = {1.0, 0, 0};
        ds.targetRelSpeed = 1.0;
        ptg->updateNavDynamicState(ds);

        // Predict the path segment:
        uint32_t ptg_step;
        bool     stepOk = ptg->getPathStepForDist(trajIdx, trajDist, ptg_step);
        if (!stepOk) continue;  // No solution with this ptg

        const auto reconstrRelPose = ptg->getPathPose(trajIdx, ptg_step);

        // tentative pose:
        const auto q = node.pose + reconstrRelPose;

        // TODO: More flexible check? Variable no. of points?
        mrpt::math::TPoint2D closestObs;
        float                closestDistSqr;
        obstacles->kdTreeClosestPoint2D({q.x, q.y}, closestObs, closestDistSqr);

        const auto closestObsWrtRobot = q.inverseComposePoint(closestObs);

        const bool isCollision = ptg->isPointInsideRobotShape(
            closestObsWrtRobot.x, closestObsWrtRobot.y);

        if (!isCollision) return q;
    }
    THROW_EXCEPTION("Could not draw collision-free random pose!");
}

TPS_RRTstar::closest_nodes_list_t TPS_RRTstar::find_nodes_within_ball(
    const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
    const double maxDistance, const TrajectoriesAndRobotShape& trs)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "find_nodes_within_ball");

    const auto& nodes = tree.nodes();
    ASSERT_(!nodes.empty());

    // Prepare distance evaluators for each PTG:
    const auto nPTGs = trs.ptgs.size();
    ASSERT_(nPTGs >= 1);

    std::vector<PoseDistanceMetric<SE2_KinState>> distEvaluators;
    for (auto& ptg : trs.ptgs) distEvaluators.emplace_back(*ptg);

    closest_nodes_list_t closestNodes;

    // TODO: Use KD-tree with nanoflann!

    for (const auto& node : nodes)
    {
        const SE2_KinState& nodeState = node.second;

        for (ptg_index_t ptgIdx = 0; ptgIdx < distEvaluators.size(); ptgIdx++)
        {
            auto& de = distEvaluators.at(ptgIdx);

            // Skip the more expensive calculation of exact distance:
            if (de.cannotBeNearerThan(nodeState, query, maxDistance))
            {
                // It's too far, skip:
                continue;
            }

            // Exact look up in the PTG manifold of poses:
            const auto ret = de.distance(nodeState, query);
            if (!ret.has_value())
            {
                // No exact solution with this ptg, skip:
                continue;
            }
            const auto [distance, trajIndex] = *ret;
            if (distance > maxDistance)
            {
                // Too far, skip:
                continue;
            }
            // Ok, accept it:
            closestNodes.emplace(
                distance, closest_nodes_list_t::mapped_type(
                              node.first, ptgIdx, trajIndex, distance));
        }
    }
    return closestNodes;
}

mrpt::maps::CPointsMap::Ptr TPS_RRTstar::cached_local_obstacles(
    const MotionPrimitivesTreeSE2& tree, const TNodeID nodeID,
    const mrpt::maps::CPointsMap& globalObstacles, double MAX_XY_DIST)
{
    // reuse?
    const auto& node = tree.nodes().at(nodeID);

    auto itOc = local_obstacles_cache_.find(nodeID);
    if (itOc != local_obstacles_cache_.end() &&
        itOc->second.globalNodePose == node.pose)
    {  // cache hit
        return itOc->second.obs;
    }

    // create/update
    auto& loc = local_obstacles_cache_[nodeID];

    loc.globalNodePose = node.pose;
    if (!loc.obs) loc.obs = mrpt::maps::CSimplePointsMap::Create();

    transform_pc_square_clipping(
        globalObstacles, mrpt::poses::CPose2D(node.pose), MAX_XY_DIST,
        *loc.obs);

    return loc.obs;
}
