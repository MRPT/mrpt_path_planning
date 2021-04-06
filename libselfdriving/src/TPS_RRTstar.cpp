/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/random/RandomGenerators.h>
#include <selfdriving/TPS_RRTstar.h>
#include <selfdriving/bestTrajectory.h>
#include <selfdriving/render_tree.h>

#include <iostream>

using namespace selfdriving;

mrpt::containers::yaml TPS_RRTstar_Parameters::as_yaml()
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    // TODO: Add comments
    MCP_SAVE(c, initialSearchRadius);
    MCP_SAVE(c, minStepLength);
    MCP_SAVE(c, maxStepLength);
    MCP_SAVE(c, maxIterations);
    MCP_SAVE(c, metricDistanceEpsilon);
    MCP_SAVE(c, drawInTPS);
    MCP_SAVE_DEG(c, headingToleranceGenerate);
    MCP_SAVE_DEG(c, headingToleranceMetric);
    MCP_SAVE(c, pathInterpolatedSegments);
    MCP_SAVE(c, saveDebugVisualizationDecimation);

    return c;
}

void TPS_RRTstar_Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_OPT(c, initialSearchRadius);
    MCP_LOAD_OPT(c, minStepLength);
    MCP_LOAD_OPT(c, maxStepLength);
    MCP_LOAD_OPT(c, maxIterations);
    MCP_LOAD_OPT(c, metricDistanceEpsilon);
    MCP_LOAD_OPT(c, drawInTPS);
    MCP_LOAD_OPT_DEG(c, headingToleranceGenerate);
    MCP_LOAD_OPT_DEG(c, headingToleranceMetric);
    MCP_LOAD_OPT(c, pathInterpolatedSegments);
    MCP_LOAD_OPT(c, saveDebugVisualizationDecimation);
}

TPS_RRTstar_Parameters TPS_RRTstar_Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    TPS_RRTstar_Parameters p;
    p.load_from_yaml(c);
    return p;
}

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
    return p.x < max.x && p.y < max.y && p.phi < max.phi + 1e-6 &&  //
           p.x > min.x && p.y > min.y && p.phi > min.phi - 1e-6;
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
    double MAX_XY_DIST = 0;
    for (const auto& ptg : in.ptgs.ptgs)
        mrpt::keep_max(MAX_XY_DIST, ptg->getRefDistance());
    ASSERT_(MAX_XY_DIST > 0);

    //  1  |  X_T ← {X_0 }    # Tree nodes (state space)
    // ------------------------------------------------------------------
    tree.root = tree.next_free_node_ID();
    tree.insert_root_node(tree.root, in.stateStart);

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    // Insert a dummy edge between root -> goal, just to allow "goal" to be
    // picked in find_reachable_nodes_from() (i.e. "tree U x_goal")
    //
    const TNodeID goalNodeId = tree.next_free_node_ID();
    po.goalNodeId            = goalNodeId;
    {
        MoveEdgeSE2_TPS dummyEdge;
        dummyEdge.cost      = std::numeric_limits<cost_t>::max();
        dummyEdge.parentId  = tree.root;
        dummyEdge.stateFrom = in.stateStart;
        dummyEdge.stateTo   = in.stateGoal;
        tree.insert_node_and_edge(
            tree.root, goalNodeId, in.stateGoal, dummyEdge);
    }

    // Dynamic search radius:
    double searchRadius = params_.initialSearchRadius;

    // Prepare draw params:
    const DrawFreePoseParams drawParams(in, tree, searchRadius, goalNodeId);

    // obstacles (TODO: dynamic over future time?):
    const auto obstaclePoints = in.obstacles->obstacles();

    //  3  |  for i \in [1,N] do
    for (size_t rrtIter = 0; rrtIter < params_.maxIterations; rrtIter++)
    {
        mrpt::system::CTimeLoggerEntry tle1(profiler_, "plan.iter");

        // 4  |   q_i ← SAMPLE( Q_free )
        // ------------------------------------------------------------------
        // (TODO: What about dynamic obstacles that depend on time?)
        const auto [qi, qiExistingID, qiNearbyNodes] =
            draw_random_free_pose(drawParams);

        //  5  |   {x_best, x_i} ← argmin{x ∈ Tree | cost[x, q_i ] < r ∧
        //  CollisionFree(pi(x,q_i)}( cost[x] + cost[x,x_i] )
        // ------------------------------------------------------------------
        const path_to_nodes_list_t closeNodes = find_source_nodes_towards(
            tree, qi, searchRadius, in.ptgs, goalNodeId, qiNearbyNodes);

        if (closeNodes.empty()) continue;  // No body around?

        // Check for CollisionFree and keep the smallest cost:
        std::optional<MoveEdgeSE2_TPS> bestEdge;
        std::optional<cost_t>          bestCost;
        size_t                         nValidCandidateSourceNodes = 0;

        for (const auto& tupl : closeNodes)
        {
            // std::tuple<TNodeID, ptg_index_t, trajectory_index_t, distance_t>
            const auto [nodeId, ptgIdx, trajIdx, trajDist] = tupl.second;

            // Do not pick "goal" as source node (!), only as target, in the
            // next rewiring step:
            if (nodeId == goalNodeId) continue;

            // Only nodes sufficiently apart from each other:
            // const distance_t dist = tupl.first;

            // do not even continue testing, draw a new q_i:
            // if (dist < params_.minStepLength)break;

            const auto& localObstacles = cached_local_obstacles(
                tree, nodeId, *obstaclePoints, MAX_XY_DIST);

            const auto&             srcNode = tree.nodes().at(nodeId);
            auto&                   ptg     = *in.ptgs.ptgs.at(ptgIdx);
            ptg_t::TNavDynamicState ds;
            (ds.curVelLocal = srcNode.vel).rotate(-srcNode.pose.phi);
            ds.relTarget      = qi - srcNode.pose;
            ds.targetRelSpeed = 1.0;
            ptg.updateNavDynamicState(ds);

            const distance_t freeDistance =
                tp_obstacles_single_path(trajIdx, *localObstacles, ptg);

            if (trajDist >= freeDistance)
            {
                // we would need to move farther away than what is possible
                // without colliding: discard this trajectory.
                continue;
            }

            // Ok, accept this motion.
            // Predict the path segment:
            uint32_t ptg_step;
            bool stepOk = ptg.getPathStepForDist(trajIdx, trajDist, ptg_step);
            if (!stepOk) continue;  // No solution with this ptg

            const auto reconstrRelPose = ptg.getPathPose(trajIdx, ptg_step);
            const auto relTwist        = ptg.getPathTwist(trajIdx, ptg_step);

            // new tentative node pose & velocity:
            const auto q_i = srcNode.pose + reconstrRelPose;

            const double headingError =
                std::abs(mrpt::math::angDistance(q_i.phi, qi.phi));
            if (headingError > params_.headingToleranceGenerate)
            {
                // Too large error in heading, skip:
                continue;
            }

            SE2_KinState x_i;
            x_i.pose = q_i;
            // relTwist is relative to the *parent* (srcNode) frame:
            (x_i.vel = relTwist).rotate(srcNode.pose.phi);

            MoveEdgeSE2_TPS tentativeEdge;
            tentativeEdge.parentId       = nodeId;
            tentativeEdge.ptgDist        = trajDist;
            tentativeEdge.ptgIndex       = ptgIdx;
            tentativeEdge.ptgPathIndex   = trajIdx;
            tentativeEdge.targetRelSpeed = ds.targetRelSpeed;
            tentativeEdge.stateFrom      = srcNode;
            tentativeEdge.stateTo        = x_i;
            // interpolated path:
            if (const auto nSeg = params_.pathInterpolatedSegments; nSeg > 0)
            {
                auto& ip = tentativeEdge.interpolatedPath.emplace();
                ip.emplace_back(0, 0, 0);  // fixed
                // interpolated:
                for (size_t i = 0; i < nSeg; i++)
                {
                    const auto iStep = ((i + 1) * ptg_step) / (nSeg + 2);
                    ip.emplace_back(ptg.getPathPose(trajIdx, iStep));
                }
                ip.emplace_back(reconstrRelPose);  // already known
            }

            // Let's compute its cost:
            tentativeEdge.cost = cost_path_segment(tentativeEdge);
            ASSERT_GT_(tentativeEdge.cost, .0);

            const cost_t cost_x           = srcNode.cost_;
            const cost_t newTentativeCost = cost_x + tentativeEdge.cost;

            ++nValidCandidateSourceNodes;

            if (!bestCost.has_value() || newTentativeCost < *bestCost)
            {
                bestCost = newTentativeCost;
                bestEdge = tentativeEdge;
            }
        }
        if (!bestEdge)
        {
            MRPT_LOG_DEBUG_STREAM(
                "iter: " << rrtIter << ", no valid edge found.");

            continue;  // no valid edge found
        }

        // Extend graph:
        //  6  |   parent[x_i] ← x_best
        //  7  |   cost[x_i] ← cost[x_best] + cost[x_best, x_i]
        // 11  |   X_T ← X_T U { x_i }
        // 12  |   E_T ← E_T U { ( x_best, x_i ) }
        // ------------------------------------------------------------------
        size_t nRewired = 0;

        const SE2_KinState& newNodeState = bestEdge->stateTo;
        TNodeID             newNodeId;
        if (!qiExistingID.has_value())
        {
            newNodeId = tree.next_free_node_ID();
            tree.insert_node_and_edge(
                bestEdge->parentId, newNodeId, newNodeState, *bestEdge);
        }
        else
        {
            newNodeId = qiExistingID.value();

            if (const cost_t newCost =
                    tree.nodes().at(bestEdge->parentId).cost_ + bestEdge->cost;
                newCost < tree.nodes().at(newNodeId).cost_)
            {
                tree.rewire_node_parent(newNodeId, *bestEdge);
                ++nRewired;
            }
        }

        // Rewire graph:
        //  8  |   for all {x ∈ Tree ∪ {x_goal } |
        //           cost[x, x_i ] < r ∧
        //           cost[x_i]+cost[x_i,x]<cost[x] ∧ CollisionFree(pi(x,x_i)} do
        //  9  |        cost[x] ← cost[x_i] + cost[x_i, x]
        // 10  |        parent[x] ← x_i
        // ------------------------------------------------------------------
        const path_to_nodes_list_t reachableNodes = find_reachable_nodes_from(
            tree, newNodeId, searchRadius, in.ptgs, qiNearbyNodes);

        // Check collisions:
        const auto& localObstaclesNewNode = cached_local_obstacles(
            tree, newNodeId, *obstaclePoints, MAX_XY_DIST);
        const auto& newNode = tree.nodes().at(newNodeId);

        for (const auto& tupl : reachableNodes)
        {
            // std::tuple<TNodeID, ptg_index_t, trajectory_index_t, distance_t>
            const auto [nodeId, ptgIdx, trajIdx, trajDist] = tupl.second;

            // We are checking edges: newNodeId ==> nodeId

            auto&                   ptg = *in.ptgs.ptgs.at(ptgIdx);
            ptg_t::TNavDynamicState ds;
            (ds.curVelLocal = newNode.vel).rotate(-newNode.pose.phi);
            MRPT_TODO("Include target node speed!");
            ds.relTarget      = {1.0, 0, 0};
            ds.targetRelSpeed = 1.0;
            ptg.updateNavDynamicState(ds);

            const distance_t freeDistance =
                tp_obstacles_single_path(trajIdx, *localObstaclesNewNode, ptg);

            if (trajDist >= freeDistance)
            {
                // we would need to move farther away than what is possible
                // without colliding: discard this trajectory.
                continue;
            }

            // Ok, accept this motion.
            // Predict the path segment:
            uint32_t ptg_step;
            bool stepOk = ptg.getPathStepForDist(trajIdx, trajDist, ptg_step);
            if (!stepOk) continue;  // No solution with this ptg

            // Ensure the path reconstruction is consistent:
            const auto reconstrRelPose = ptg.getPathPose(trajIdx, ptg_step);
            {
                const auto relTwist = ptg.getPathTwist(trajIdx, ptg_step);

                // new tentative node pose & velocity:
                const auto   q_i = newNode.pose + reconstrRelPose;
                SE2_KinState x_i;
                x_i.pose = q_i;
                // relTwist is relative to the *parent* (srcNode) frame:
                (x_i.vel = relTwist).rotate(newNode.pose.phi);

                MRPT_TODO("Do the check here.");
            }

            const auto& trgNode = tree.nodes().at(nodeId);

            MoveEdgeSE2_TPS rewiredEdge;
            rewiredEdge.parentId       = newNodeId;
            rewiredEdge.ptgDist        = trajDist;
            rewiredEdge.ptgIndex       = ptgIdx;
            rewiredEdge.ptgPathIndex   = trajIdx;
            rewiredEdge.targetRelSpeed = ds.targetRelSpeed;
            rewiredEdge.stateFrom      = newNodeState;
            rewiredEdge.stateTo        = trgNode;
            // interpolated path:
            if (const auto nSeg = params_.pathInterpolatedSegments; nSeg > 0)
            {
                auto& ip = rewiredEdge.interpolatedPath.emplace();
                ip.emplace_back(0, 0, 0);  // fixed
                // interpolated:
                for (size_t i = 0; i < nSeg; i++)
                {
                    const auto iStep = ((i + 1) * ptg_step) / (nSeg + 2);
                    ip.emplace_back(ptg.getPathPose(trajIdx, iStep));
                }
                ip.emplace_back(reconstrRelPose);  // already known
            }

            // Let's compute the tentative cost of rewiring the tree
            // such that `srcNode` is more easily reachable from `newNode`:
            rewiredEdge.cost              = cost_path_segment(rewiredEdge);
            const cost_t curCost          = trgNode.cost_;
            const cost_t newTentativeCost = newNode.cost_ + rewiredEdge.cost;

            if (newTentativeCost < curCost)
            {
                // Yes, do rewire:
                // MRPT_LOG_DEBUG_STREAM("Rewiring node " << nodeId<< "
                // newCost=" << newTentativeCost << " < "<< "curCost=" <<
                // curCost);
                ++nRewired;
                tree.rewire_node_parent(nodeId, rewiredEdge);
            }
        }

        MRPT_LOG_DEBUG_FMT(
            "iter: %5u qi=%40s candidates/evaluated/rewired= %3u/%3u/%3u",
            static_cast<unsigned int>(rrtIter), qi.asString().c_str(),
            static_cast<unsigned int>(closeNodes.size()),
            static_cast<unsigned int>(nValidCandidateSourceNodes),
            static_cast<unsigned int>(nRewired));

        // Debug log files:
        if (params_.saveDebugVisualizationDecimation > 0 &&
            (rrtIter % params_.saveDebugVisualizationDecimation) == 0)
        {
            RenderOptions ro;
            ro.highlight_path_to_node_id = newNodeId;
            mrpt::opengl::COpenGLScene scene;
            scene.insert(render_tree(tree, in, ro));
            scene.saveToFile(mrpt::format(
                "debug_rrtstar_%05u.3Dscene",
                static_cast<unsigned int>(rrtIter)));
        }

    }  // for each rrtIter

    // RRT ended, now collect the result:
    // ----------------------------------------
    const auto foundPath      = tree.backtrack_path(goalNodeId);
    bool       foundPathValid = true;

    for (const auto& step : foundPath)
    {
        if (step.cost_ == std::numeric_limits<cost_t>::max())
        {
            foundPathValid = false;
            break;
        }
    }
    po.success = foundPathValid;

    po.pathCost = tree.nodes().at(goalNodeId).cost_;

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

TPS_RRTstar::draw_pose_return_t TPS_RRTstar::draw_random_free_pose(
    const TPS_RRTstar::DrawFreePoseParams& p)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "draw_random_free_pose");

    if (params_.drawInTPS)
        return draw_random_tps(p);
    else
        return draw_random_euclidean(p);
}

TPS_RRTstar::draw_pose_return_t TPS_RRTstar::draw_random_euclidean(
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

        closest_lie_nodes_list_t closeNodes =
            find_nearby_nodes(p.tree_, q, p.searchRadius_ * 1.2);

        const double minFoundDistance = closeNodes.empty()
                                            ? params_.metricDistanceEpsilon
                                            : closeNodes.begin()->first;

        // Do we have a match with an existing node ID?
        if (minFoundDistance < params_.metricDistanceEpsilon)
        {
            // Return a match with an existing node ID:
            const auto existingId = closeNodes.begin()->second.get().nodeID_;
            closeNodes.erase(closeNodes.begin());
            return {q, existingId, closeNodes};
        }

        // TODO: More flexible check? Variable no. of points?
        mrpt::math::TPoint2D closestObs;
        float                closestDistSqr;
        obstacles->kdTreeClosestPoint2D({q.x, q.y}, closestObs, closestDistSqr);

        const auto closestObsWrtRobot = q.inverseComposePoint(closestObs);

        const bool isCollision =
            selfdriving::obstaclePointCollides(closestObsWrtRobot, p.pi_.ptgs);

        if (!isCollision) return {q, std::nullopt, closeNodes};
    }
    THROW_EXCEPTION("Could not draw collision-free random pose!");
}

TPS_RRTstar::draw_pose_return_t TPS_RRTstar::draw_random_tps(
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
        const auto nodeIdx = rng.drawUniform32bit() % p.tree_.nodes().size();

        // Cannot pick the dummy goal node as *start* pose, it's only a *final*
        // node:
        if (nodeIdx == p.goalNodeId_) continue;

        const auto& node = p.tree_.nodes().at(nodeIdx);

        const auto  ptgIdx = rng.drawUniform32bit() % p.pi_.ptgs.ptgs.size();
        const auto& ptg    = p.pi_.ptgs.ptgs.at(ptgIdx);

        const auto trajIdx =
            rng.drawUniform32bit() % ptg->getAlphaValuesCount();
        const auto trajDist = rng.drawUniform(
            params_.minStepLength,
            std::min(params_.maxStepLength, p.searchRadius_));

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

        // within bounding box?
        if (q.x < p.pi_.worldBboxMin.x || q.y < p.pi_.worldBboxMin.y ||
            q.phi < p.pi_.worldBboxMin.phi || q.x > p.pi_.worldBboxMax.x ||
            q.y > p.pi_.worldBboxMax.y || q.phi > p.pi_.worldBboxMax.phi)
        {
            // Out of allowed space:
            continue;
        }

        // Check: minimum distance to any other pose:
        // In this case, do NOT use TPS, but the real SE(2) metric space,
        // to avoid the lack of existing paths to hide nodes that are really
        // close to this tentative pose sample:

        closest_lie_nodes_list_t closeNodes =
            find_nearby_nodes(p.tree_, q, p.searchRadius_);

        //        const auto [minFoundDistance, closestNodeId]
        //        =find_closest_node(p.tree_, q);

        // Match with existing node?
        if (!closeNodes.empty() &&
            closeNodes.begin()->first < params_.metricDistanceEpsilon)
        {
            // Return the ID of the existing node so we can reconsider it:
            const auto closestNodeId = closeNodes.begin()->second.get().nodeID_;
            closeNodes.erase(closeNodes.begin());

            return {q, closestNodeId, closeNodes};
        }

        // Approximate check for collisions:
        // TODO: More flexible check? Variable no. of points?
        mrpt::math::TPoint2D closestObs;
        float                closestDistSqr;
        obstacles->kdTreeClosestPoint2D({q.x, q.y}, closestObs, closestDistSqr);

        const auto closestObsWrtRobot = q.inverseComposePoint(closestObs);

        const bool isCollision = ptg->isPointInsideRobotShape(
            closestObsWrtRobot.x, closestObsWrtRobot.y);

        if (!isCollision)
        {
            // Ok, good sample has been drawn:
            closest_lie_nodes_list_t closeNodes =
                find_nearby_nodes(p.tree_, q, p.searchRadius_ * 1.2);

            return {q, std::nullopt, closeNodes};
        }
    }
    THROW_EXCEPTION("Could not draw collision-free random pose!");
}

// See docs in .h
TPS_RRTstar::path_to_nodes_list_t TPS_RRTstar::find_source_nodes_towards(
    const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
    const double maxDistance, const TrajectoriesAndRobotShape& trs,
    const TNodeID                   goalNodeToIgnore,
    const closest_lie_nodes_list_t& hintCloseNodes)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "find_source_nodes_towards");

    const auto& nodes = tree.nodes();
    ASSERT_(!nodes.empty());

    // Prepare distance evaluators for each PTG:
    const auto nPTGs = trs.ptgs.size();
    ASSERT_(nPTGs >= 1);

    std::vector<PoseDistanceMetric_TPS<SE2_KinState>> distEvaluators;
    for (auto& ptg : trs.ptgs)
        distEvaluators.emplace_back(*ptg, params_.headingToleranceMetric);

    path_to_nodes_list_t closestNodes;

    for (const auto& distNodeId : hintCloseNodes)
    {
        const auto nodeId = distNodeId.second.get().nodeID_;

        if (nodeId == goalNodeToIgnore) continue;  // ignore

        // Don't take into account too short segments:
        // if (distNodeId.first < params_.minStepLength) continue;

        const SE2_KinState& nodeState = tree.nodes().at(nodeId);

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
            ASSERTMSG_(distance > 0, "Repeated pose node in tree?");

            if (nodeState.pose == query)
            {
                std::cout << "repeated: dist=" << distance << "\n";
                ASSERTMSG_(false, "Repeated pose node in tree? (bis)");
            }

            if (distance > maxDistance)
            {
                // Too far, skip:
                continue;
            }
            // Ok, accept it:
            closestNodes.emplace(
                distance, path_to_nodes_list_t::mapped_type(
                              nodeId, ptgIdx, trajIndex, distance));
        }
    }
    return closestNodes;
}

// See docs in .h
// This is mostly similar to find_source_nodes_towards(), but with
// reversed order between source and target states.
TPS_RRTstar::path_to_nodes_list_t TPS_RRTstar::find_reachable_nodes_from(
    const MotionPrimitivesTreeSE2& tree, const TNodeID queryNodeId,
    const double maxDistance, const TrajectoriesAndRobotShape& trs,
    const closest_lie_nodes_list_t& hintCloseNodes)
{
    auto tle =
        mrpt::system::CTimeLoggerEntry(profiler_, "find_reachable_nodes_from");

    const auto& nodes = tree.nodes();
    ASSERT_(!nodes.empty());

    const auto& query = nodes.at(queryNodeId);

    MRPT_TODO("Build list of nearby nodes once between these two methods");

    // Prepare distance evaluators for each PTG:
    const auto nPTGs = trs.ptgs.size();
    ASSERT_(nPTGs >= 1);

    std::vector<PoseDistanceMetric_TPS<SE2_KinState>> distEvaluators;
    for (auto& ptg : trs.ptgs)
        distEvaluators.emplace_back(*ptg, params_.headingToleranceMetric);

    path_to_nodes_list_t closestNodes;

    for (const auto& distNodeId : hintCloseNodes)
    {
        const auto& node   = distNodeId.second.get();
        const auto  nodeId = node.nodeID_;

        const SE2_KinState& nodeState = node;

        // Don't rewire to myself ;-)
        if (nodeId == queryNodeId) continue;

        // Don't take into account too short segments:
        if (distNodeId.first < params_.minStepLength) continue;

        for (ptg_index_t ptgIdx = 0; ptgIdx < distEvaluators.size(); ptgIdx++)
        {
            auto& de = distEvaluators.at(ptgIdx);

            // Skip the more expensive calculation of exact distance:
            if (de.cannotBeNearerThan(query, nodeState.pose, maxDistance))
            {
                // It's too far, skip:
                continue;
            }

            // Exact look up in the PTG manifold of poses:
            MRPT_TODO("Target velocity not accounted for!!!");
            const auto ret = de.distance(/*from*/ query, /*to*/ nodeState.pose);
            if (!ret.has_value())
            {
                // No exact solution with this ptg, skip:
                continue;
            }
            const auto [distance, trajIndex] = *ret;
            ASSERTMSG_(distance > 0, "Repeated pose node in tree?");

            if (distance > maxDistance)
            {
                // Too far, skip:
                continue;
            }
            // Ok, accept it:
            closestNodes.emplace(
                distance, path_to_nodes_list_t::mapped_type(
                              nodeId, ptgIdx, trajIndex, distance));
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

cost_t TPS_RRTstar::cost_path_segment(const MoveEdgeSE2_TPS& edge) const
{
    return edge.ptgDist;
}

TPS_RRTstar::closest_lie_nodes_list_t TPS_RRTstar::find_nearby_nodes(
    const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query,
    const double maxDistance)
{
    auto tle = mrpt::system::CTimeLoggerEntry(profiler_, "find_nearby_nodes");

    closest_lie_nodes_list_t             out;
    PoseDistanceMetric_Lie<SE2_KinState> de;

    // TODO: Use KD-tree with nanoflann!
    for (const auto& node : tree.nodes())
    {
        const SE2_KinState& nodeState = node.second;

        // Skip the more expensive calculation of exact distance:
        if (de.cannotBeNearerThan(query, nodeState.pose, maxDistance))
            continue;  // It's too far, skip:

        if (auto d = de.distance(query, nodeState.pose); d < maxDistance)
            out.emplace(d, std::ref(node.second));
    }
    return out;
}

std::tuple<distance_t, TNodeID> TPS_RRTstar::find_closest_node(
    const MotionPrimitivesTreeSE2& tree, const mrpt::math::TPose2D& query) const
{
    ASSERT_(!tree.nodes().empty());

    distance_t minDist = std::numeric_limits<distance_t>::max();
    TNodeID    closestNodeId;
    PoseDistanceMetric_Lie<SE2_KinState> de;

    for (const auto& node : tree.nodes())
    {
        const SE2_KinState& nodeState = node.second;

        // Skip the more expensive calculation of exact distance:
        if (de.cannotBeNearerThan(query, nodeState.pose, minDist))
            continue;  // It's too far, skip:

        const auto d = de.distance(query, nodeState.pose);
        if (d < minDist)
        {
            closestNodeId = node.first;
            minDist       = d;
        }
    }
    return {minDist, closestNodeId};
}
