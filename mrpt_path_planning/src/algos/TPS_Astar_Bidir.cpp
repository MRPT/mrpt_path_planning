/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/TPS_Astar_Bidir.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/tp_obstacles_single_path.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/version.h>

#include <map>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(TPS_Astar_Bidir, Planner, mpp)

using namespace mpp;

TPS_Astar_Bidir::TPS_Astar_Bidir() { profiler_().setName("TPS_Astar_Bidir"); }

// ---------------------------------------------------------------------------
// Backward neighbor generation (predecessors).
// ---------------------------------------------------------------------------
TPS_Astar::list_paths_to_neighbors_t
    TPS_Astar_Bidir::find_feasible_predecessors(
        const Node& from, const TrajectoriesAndRobotShape& trs,
        const SE2orR2_KinState&                         startState,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                     MAX_XY_OBSTACLES_CLIPPING_DIST,
        const mrpt::math::TPose2D& worldBboxMin,
        const mrpt::math::TPose2D& worldBboxMax)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "find_feasible_pred");

    const auto   qPose    = from.state.pose;  // the node being expanded
    const double halfCell = params_.grid_resolution_xy * 0.5;

    const mrpt::math::TPose2D startPose = startState.asSE2KinState().pose;

    // Keep the cheapest predecessor per lattice cell:
    std::unordered_map<NodeCoords, path_to_neighbor_t, NodeCoordsHash>
        bestPaths;

    // Cache of the TP-obstacle free-distance array, keyed by (ptgIdx, predCell):
    // unlike forward expansion (single obstacle anchor = the node), each
    // predecessor pose is a different obstacle anchor, so we cache per cell to
    // avoid re-scanning the local cloud for predecessors that share a cell.
    std::map<std::pair<size_t, std::pair<int32_t, int32_t>>, std::vector<double>>
        tpObsCache;

    for (size_t ptgIdx = 0; ptgIdx < trs.ptgs.size(); ptgIdx++)
    {
        auto& ptg = trs.ptgs.at(ptgIdx);
        ASSERT_(ptg->isInitialized());

        auto ptgTrimmable =
            std::dynamic_pointer_cast<ptg::SpeedTrimmablePTG>(ptg);

        const duration_seconds_t ptg_dt = ptg->getPathStepDuration();

        // Build the trajectory-index subset to consider (same policy as the
        // forward generator):
        std::set<trajectory_index_t> trajIdxsToConsider;
        ASSERT_(params_.max_ptg_trajectories_to_explore >= 2);
        for (size_t i = 0; i < params_.max_ptg_trajectories_to_explore; i++)
        {
            trajectory_index_t trjIdx = mrpt::round(
                i * (ptg->getPathCount() - 1) /
                (params_.max_ptg_trajectories_to_explore - 1));
            trajIdxsToConsider.insert(trjIdx);
        }

        std::vector<normalized_speed_t> speedsToConsider;
        if (ptgTrimmable)
        {
            ASSERT_(params_.max_ptg_speeds_to_explore >= 1);
            normalized_speed_t speedStep =
                1.0 / params_.max_ptg_speeds_to_explore;
            for (normalized_speed_t s = speedStep; s < 1.001; s += speedStep)
                speedsToConsider.push_back(s);
        }
        else { speedsToConsider.push_back(1.0); }

        for (const auto speed : speedsToConsider)
        {
            if (ptgTrimmable) ptgTrimmable->trimmableSpeed_ = speed;

            for (const auto trjIdx : trajIdxsToConsider)
            {
                for (duration_seconds_t t : params_.ptg_sample_timestamps)
                {
                    const ptg_step_t trjStep = mrpt::round(t / ptg_dt);

                    const auto maxSteps = ptg->getPathStepCount(trjIdx);
                    ASSERT_(maxSteps >= 1);
                    if (trjStep == 0 || trjStep >= maxSteps) continue;

                    // The forward PTG arc, in the predecessor-local frame:
                    const auto relPose = ptg->getPathPose(trjIdx, trjStep);
                    const auto relDist = ptg->getPathDist(trjIdx, trjStep);

                    // Predecessor pose: p such that p (+) relPose == qPose
                    //   CPose2D(p) = CPose2D(q) (+) inverse(relPose)
                    const auto pPose =
                        (mrpt::poses::CPose2D(qPose) -
                         mrpt::poses::CPose2D(relPose))
                            .asTPose();

                    // out of lattice limits?
                    if (pPose.x < worldBboxMin.x || pPose.y < worldBboxMin.y)
                        continue;
                    if (pPose.x > worldBboxMax.x - halfCell ||
                        pPose.y > worldBboxMax.y - halfCell)
                        continue;

                    const NodeCoords pCell = nodeGridCoords(pPose);

                    // TP-obstacles for a forward arc *from p*: build (and cache)
                    // the free-distance array anchored at the predecessor pose.
                    const auto cacheKey = std::make_pair(
                        ptgIdx, std::make_pair(pCell.idxX, pCell.idxY));
                    auto itCache = tpObsCache.find(cacheKey);
                    if (itCache == tpObsCache.end())
                    {
                        const auto localObstacles = cached_local_obstacles(
                            pPose, globalObstacles,
                            MAX_XY_OBSTACLES_CLIPPING_DIST);

                        ptg_t::TNavDynamicState ds;
                        ds.curVelLocal = mrpt::math::TTwist2D(0, 0, 0);
                        ds.relTarget   = startPose - pPose;
                        ds.targetRelSpeed = 0;
                        ptg->updateNavDynamicState(ds);

                        std::vector<double> tpObstacles;
                        ptg->initTPObstacles(tpObstacles);
                        const auto&  ox   = localObstacles->getPointsBufferRef_x();
                        const auto&  oy   = localObstacles->getPointsBufferRef_y();
                        const size_t nObs = localObstacles->size();
                        for (size_t i = 0; i < nObs; i++)
                            ptg->updateTPObstacle(ox[i], oy[i], tpObstacles);

                        itCache =
                            tpObsCache.emplace(cacheKey, std::move(tpObstacles))
                                .first;
                    }

                    const distance_t freeDistance = itCache->second[trjIdx];
                    if (relDist >= freeDistance) continue;  // collision

                    // Valid predecessor. Store with forward-generator
                    // convention: neighborPose = from.pose (+) relReconstrPose
                    //   => relReconstrPose = inverse(relPose)
                    const NodeCoords nc = pCell;
                    auto&            path = bestPaths[nc];
                    if (relDist < path.ptgDist)
                    {
                        path.ptgDist      = relDist;
                        path.ptgIndex     = ptgIdx;
                        path.ptgTrajIndex = trjIdx;
                        path.relTrgStep   = trjStep;
                        path.relReconstrPose =
                            (mrpt::poses::CPose2D(pPose) -
                             mrpt::poses::CPose2D(qPose))
                                .asTPose();
                        path.neighborNodeCoords = nc;
                        path.ptgDynState        = ptg->getCurrentNavDynamicState();
                        path.ptgTrimmableSpeed  = speed;
                    }
                }
            }
        }
    }

    list_paths_to_neighbors_t neighbors;
    for (const auto& kv : bestPaths)
    {
        if (!kv.second.ptgIndex.has_value()) continue;
        neighbors.emplace_back(kv.second);
    }
    return neighbors;
}

// ---------------------------------------------------------------------------
// Bidirectional A*
// ---------------------------------------------------------------------------
PlannerOutput TPS_Astar_Bidir::plan(const PlannerInput& in)
{
    MRPT_START
    mrpt::system::CTimeLoggerEntry tleg(profiler_(), "plan");

    const double planInitTime = mrpt::Clock::nowDouble();

    // Sanity checks (mirror TPS_Astar::plan):
    ASSERT_(in.ptgs.initialized());
    ASSERT_(in.worldBboxMin != in.worldBboxMax);
    ASSERT_(!in.stateGoal.state.isEmpty());

    PlannerOutput po;
    po.originalInput = in;
    auto& tree       = po.motionTree;
    tree.edges_to_children.clear();

    // clipping dist + max speed:
    double MAX_XY_DIST = 0;
    maxLinSpeed_       = 0.0;
    for (const auto& ptg : in.ptgs.ptgs)
    {
        mrpt::keep_max(MAX_XY_DIST, ptg->getRefDistance());
        mrpt::keep_max(maxLinSpeed_, ptg->getMaxLinVel());
    }
    ASSERT_(MAX_XY_DIST > 0);
    if (maxLinSpeed_ <= 0) maxLinSpeed_ = 1.0;

    // Warn for non-constant-velocity PTGs (see class restrictions):
    for (const auto& ptg : in.ptgs.ptgs)
    {
        if (std::dynamic_pointer_cast<ptg::SpeedTrimmablePTG>(ptg))
        {
            MRPT_LOG_WARN(
                "TPS_Astar_Bidir: speed-trimmable PTG detected. The "
                "bidirectional join assumes piecewise-constant velocity "
                "(C-PTG); velocity continuity at the meeting point is not "
                "guaranteed.");
            break;
        }
    }

    // obstacles:
    std::vector<mrpt::maps::CPointsMap::Ptr> obstaclePoints;
    for (const auto& os : in.obstacles)
    {
        if (!os) continue;
        os->apply_clipping_box(mrpt::math::TBoundingBox(
            {in.worldBboxMin.x, in.worldBboxMin.y, -1.0},
            {in.worldBboxMax.x, in.worldBboxMax.y, 1.0}));
        obstaclePoints.emplace_back(os->obstacles());
    }

    // Two lattices + open sets:
    SE2_Lattice gridFwd, gridBwd;
    localObstaclesCache_.clear();

    mrpt::graphs::TNodeID nextFwdId = 0, nextBwdId = 0;

    // Edge to (tree-)parent, keyed by child node id, per side:
    std::unordered_map<mrpt::graphs::TNodeID, MoveEdgeSE2_TPS> edgeFwd, edgeBwd;

    std::multimap<cost_t, Node*> openFwd, openBwd;

    // Backward "goal" for the forward heuristic = the actual goal:
    const SE2orR2_KinState& goalState = in.stateGoal;
    // Forward "goal" for the backward heuristic = the start pose (SE2):
    SE2orR2_KinState startState;
    startState.state = in.stateStart.pose;

    auto getOrCreate = [&](SE2_Lattice& grid, const SE2_KinState& st,
                           mrpt::graphs::TNodeID& nextId) -> Node&
    {
        auto& n = grid[nodeGridCoords(st.pose)];
        if (!n.id.has_value())
        {
            n.id    = nextId++;
            n.state = st;
        }
        return n;
    };

    // Seed forward root = start:
    {
        auto& n            = getOrCreate(gridFwd, in.stateStart, nextFwdId);
        n.state            = in.stateStart;
        n.gScore           = 0;
        n.fScore           = heuristic(n.state, goalState);
        n.pendingInOpenSet = true;
        openFwd.insert({n.fScore, &n});
    }

    // Seed backward root(s):
    if (in.stateGoal.state.isPose())
    {
        SE2_KinState gs;
        gs.pose            = in.stateGoal.state.pose();
        gs.vel             = in.stateGoal.vel;
        auto& n            = getOrCreate(gridBwd, gs, nextBwdId);
        n.state            = gs;
        n.gScore           = 0;
        n.fScore           = heuristic(n.state, startState);
        n.pendingInOpenSet = true;
        openBwd.insert({n.fScore, &n});
    }
    else
    {
        // R(2) point goal: one backward root per yaw bin (multi-source).
        const auto& gp     = in.stateGoal.state.point();
        const int   nBins  = static_cast<int>(
            std::round(2 * M_PI / params_.grid_resolution_yaw));
        for (int j = 0; j < nBins; j++)
        {
            SE2_KinState gs;
            gs.pose = {gp.x, gp.y, j * params_.grid_resolution_yaw};
            auto& n = getOrCreate(gridBwd, gs, nextBwdId);
            if (n.gScore == 0 && n.pendingInOpenSet) continue;  // dup bin
            n.state            = gs;
            n.gScore           = 0;
            n.fScore           = heuristic(n.state, startState);
            n.pendingInOpenSet = true;
            openBwd.insert({n.fScore, &n});
        }
    }

    // Best meeting found so far:
    cost_t                bestMu     = std::numeric_limits<cost_t>::max();
    mrpt::graphs::TNodeID meetFwdId  = mrpt::graphs::INVALID_NODEID;
    mrpt::graphs::TNodeID meetBwdId  = mrpt::graphs::INVALID_NODEID;
    NodeCoords            meetCell;
    bool                  haveMeet = false;

    auto topF = [](const std::multimap<cost_t, Node*>& os) -> cost_t
    {
        // skip stale (visited) entries are handled at pop; here we just peek:
        return os.empty() ? std::numeric_limits<cost_t>::max()
                          : os.begin()->first;
    };

    // Relax a generated neighbor/predecessor edge. Returns nothing; updates the
    // grid/open set of the given side.
    auto relax =
        [&](Direction dir, Node& current, SE2_Lattice& grid,
            std::multimap<cost_t, Node*>& openSet,
            std::unordered_map<mrpt::graphs::TNodeID, MoveEdgeSE2_TPS>& edgeMap,
            mrpt::graphs::TNodeID& nextId, const SE2orR2_KinState& sideGoal,
            const path_to_neighbor_t& edge)
    {
        auto& ptg = *in.ptgs.ptgs.at(edge.ptgIndex.value());
        ptg.updateNavDynamicState(edge.ptgDynState.value());
        if (auto* ptgTrim = dynamic_cast<ptg::SpeedTrimmablePTG*>(&ptg);
            ptgTrim)
            ptgTrim->trimmableSpeed_ = edge.ptgTrimmableSpeed;

        const uint32_t ptg_step = edge.relTrgStep.value();

        // Neighbor pose (predecessor for backward, successor for forward):
        const auto neighPose = current.state.pose + edge.relReconstrPose;

        if (neighPose.x < in.worldBboxMin.x || neighPose.y < in.worldBboxMin.y)
            return;
        if (neighPose.x > in.worldBboxMax.x || neighPose.y > in.worldBboxMax.y)
            return;

        SE2_KinState x_i;
        x_i.pose = neighPose;
        // Velocity (constant-vel C-PTG): twist at the relevant arc endpoint.
        const auto relTwist =
            ptg.getPathTwist(edge.ptgTrajIndex.value(), ptg_step);
        (x_i.vel = relTwist).rotate(neighPose.phi);

        auto& neighborNode = getOrCreate(grid, x_i, nextId);
        if (neighborNode.visited) return;

        // Build the tentative edge as a *forward-travel* PTG arc:
        //  - forward side:  parent=current -> child=neighbor (cur -> neigh)
        //  - backward side: the real arc is predecessor(neigh) -> current,
        //    so stateFrom=neighbor, stateTo=current.
        MoveEdgeSE2_TPS newEdge;
        newEdge.ptgIndex          = edge.ptgIndex.value();
        newEdge.ptgPathIndex      = edge.ptgTrajIndex.value();
        newEdge.ptgDist           = edge.ptgDist;
        newEdge.ptgStepIndex      = ptg_step;
        newEdge.ptgTrimmableSpeed = edge.ptgTrimmableSpeed;
        newEdge.ptgFinalGoalRelSpeed = edge.ptgDynState.value().targetRelSpeed;
#if MRPT_VERSION >= 0x20e02
        newEdge.ptgInternalState =
            ptg.getCurrentNavDynamicState().internalState;
#endif
        mrpt::math::TPose2D arcFrom, arcTo;
        if (dir == Direction::Forward)
        {
            arcFrom         = current.state.pose;
            arcTo           = neighPose;
            newEdge.parentId = current.id.value();
        }
        else
        {
            arcFrom         = neighPose;       // predecessor
            arcTo           = current.state.pose;
            newEdge.parentId = current.id.value();
        }
        newEdge.stateFrom = SE2_KinState();
        newEdge.stateFrom.pose = arcFrom;
        newEdge.stateTo        = SE2_KinState();
        newEdge.stateTo.pose   = arcTo;
        newEdge.ptgFinalRelativeGoal =
            sideGoal.asSE2KinState().pose - arcFrom;

        const auto reconstrRelPose = arcTo - arcFrom;
        edge_interpolated_path(
            newEdge, in.ptgs, reconstrRelPose, ptg_step,
            params_.pathInterpolatedSegments);
        newEdge.estimatedExecTime = ptg_step * ptg.getPathStepDuration();
        newEdge.cost              = cost_path_segment(newEdge);
        ASSERT_GT_(newEdge.cost, .0);

        const cost_t tentative_g = current.gScore + newEdge.cost;
        if (tentative_g >= neighborNode.gScore) return;

        neighborNode.cameFrom        = &current;
        neighborNode.gScore          = tentative_g;
        const cost_t costToGoal      = heuristic(x_i, sideGoal);
        neighborNode.fScore          = tentative_g + costToGoal;
        neighborNode.pendingInOpenSet = true;
        neighborNode.state            = x_i;
        edgeMap[neighborNode.id.value()] = newEdge;
        openSet.insert({neighborNode.fScore, &neighborNode});
    };

    // Meeting check: after settling node `u` on `side`, see if the opposite
    // lattice has a visited node in the same cell.
    auto checkMeet = [&](Node& u, SE2_Lattice& otherGrid, Direction uSide)
    {
        const NodeCoords c = nodeGridCoords(u.state.pose);
        auto             it = otherGrid.find(c);
        if (it == otherGrid.end()) return;
        Node& w = it->second;
        if (!w.visited || !w.id.has_value()) return;
        const cost_t mu = u.gScore + w.gScore;
        if (mu >= bestMu) return;
        bestMu   = mu;
        meetCell = c;
        haveMeet = true;
        if (uSide == Direction::Forward)
        {
            meetFwdId = u.id.value();
            meetBwdId = w.id.value();
        }
        else
        {
            meetFwdId = w.id.value();
            meetBwdId = u.id.value();
        }
    };

    unsigned int nIter = 0;
    while (!openFwd.empty() || !openBwd.empty())
    {
        // Termination: no remaining frontier can improve on the best meeting.
        const cost_t tf = topF(openFwd);
        const cost_t tb = topF(openBwd);
        if (haveMeet && bestMu <= tf && bestMu <= tb) break;

        // Pick the side with the smaller top-f (ties -> forward), as long as it
        // is non-empty.
        const bool expandForward =
            !openFwd.empty() && (openBwd.empty() || tf <= tb);

        auto& openSet = expandForward ? openFwd : openBwd;
        if (openSet.empty()) break;

        Node& current = *openSet.begin()->second;
        if (current.visited)
        {
            openSet.erase(openSet.begin());
            continue;
        }
        current.pendingInOpenSet = false;
        current.visited          = true;
        openSet.erase(openSet.begin());
        nIter++;

        // Meeting detection on settle:
        if (expandForward)
            checkMeet(current, gridBwd, Direction::Forward);
        else
            checkMeet(current, gridFwd, Direction::Backward);

        // Expand:
        if (expandForward)
        {
            const auto neighbors = find_feasible_paths_to_neighbors(
                current, in.ptgs, goalState, obstaclePoints, MAX_XY_DIST,
                nodes_with_desired_speed_t(), in.worldBboxMin, in.worldBboxMax);
            for (const auto& e : neighbors)
                relax(
                    Direction::Forward, current, gridFwd, openFwd, edgeFwd,
                    nextFwdId, goalState, e);
        }
        else
        {
            const auto preds = find_feasible_predecessors(
                current, in.ptgs, startState, obstaclePoints, MAX_XY_DIST,
                in.worldBboxMin, in.worldBboxMax);
            for (const auto& e : preds)
                relax(
                    Direction::Backward, current, gridBwd, openBwd, edgeBwd,
                    nextBwdId, startState, e);
        }

        // Timeout:
        if ((mrpt::Clock::nowDouble() - planInitTime) >
            params_.maximumComputationTime)
        {
            MRPT_LOG_DEBUG("TPS_Astar_Bidir: timeout.");
            break;
        }
    }

    po.computationTime = mrpt::Clock::nowDouble() - planInitTime;

    if (!haveMeet)
    {
        po.success = false;
        return po;
    }

    // -----------------------------------------------------------------------
    // Reconstruct + stitch + refine.
    // -----------------------------------------------------------------------
    // Forward half: walk cameFrom from meetFwd up to the forward root.
    std::vector<MoveEdgeSE2_TPS> fwdEdges;  // travel order start -> meetFwd
    {
        std::vector<MoveEdgeSE2_TPS> rev;
        const Node*                  meet = nullptr;
        for (auto& kv : gridFwd)
            if (kv.second.id == meetFwdId) { meet = &kv.second; break; }
        ASSERT_(meet != nullptr);
        const Node* node = meet;
        while (node->cameFrom.has_value())
        {
            rev.push_back(edgeFwd.at(node->id.value()));
            node = node->cameFrom.value();
        }
        fwdEdges.assign(rev.rbegin(), rev.rend());
    }

    // Backward half: walk cameFrom from meetBwd up to a backward root; each
    // stored edge is already a forward-travel arc toward the goal.
    std::vector<MoveEdgeSE2_TPS> bwdEdges;  // travel order meetBwd -> goal
    {
        const Node* meet = nullptr;
        for (auto& kv : gridBwd)
            if (kv.second.id == meetBwdId) { meet = &kv.second; break; }
        ASSERT_(meet != nullptr);
        const Node* node = meet;
        while (node->cameFrom.has_value())
        {
            bwdEdges.push_back(edgeBwd.at(node->id.value()));
            node = node->cameFrom.value();
        }
    }

    // Build the exact node-pose chain and the matching edge vector.
    // Junction: forward half ends at meetFwd pose; backward half starts at
    // meetBwd pose (same cell, sub-cell gap). We snap them together and let
    // refine_trajectory() recompute the PTG parameters of every edge from the
    // exact node poses, absorbing the gap.
    std::vector<MotionPrimitivesTreeSE2::node_t> chainNodes;
    std::vector<MoveEdgeSE2_TPS>                 chainEdges;

    auto pushNode = [&](const mrpt::math::TPose2D& p)
    {
        MotionPrimitivesTreeSE2::node_t n;
        n.pose = p;
        chainNodes.push_back(n);
    };

    pushNode(in.stateStart.pose);
    for (const auto& e : fwdEdges)
    {
        chainEdges.push_back(e);
        pushNode(e.stateTo.pose);
    }
    // backward edges; skip the sub-cell junction by continuing from meetFwd.
    for (const auto& e : bwdEdges)
    {
        chainEdges.push_back(e);
        pushNode(e.stateTo.pose);
    }

    if (chainEdges.empty())
    {
        // Degenerate: the two frontiers met at the root, i.e. the start cell is
        // already the goal cell. Emit a trivial single-node success (zero-cost,
        // empty path), consistent with how the forward planner terminates when
        // the start already satisfies the goal.
        tree.root = 0;
        tree.insert_root_node(tree.root, chainNodes.front());
        po.goalNodeId           = tree.root;
        po.bestNodeId           = tree.root;
        po.bestNodeIdCostToGoal = 0;
        po.pathCost             = 0;
        po.success              = true;
        return po;
    }

    ASSERT_EQUAL_(chainNodes.size(), chainEdges.size() + 1);

    refine_trajectory(chainNodes, chainEdges, in.ptgs);

    // Emit a fresh linear result tree start -> ... -> goal.
    mrpt::graphs::TNodeID id = 0;
    tree.root                = id;
    tree.insert_root_node(tree.root, chainNodes.front());
    mrpt::graphs::TNodeID prevId = id;
    for (size_t i = 0; i < chainEdges.size(); i++)
    {
        const mrpt::graphs::TNodeID childId = ++id;
        auto&                       e       = chainEdges[i];
        e.parentId                          = prevId;
        e.stateFrom.pose                    = chainNodes[i].pose;
        e.stateTo.pose                      = chainNodes[i + 1].pose;
        tree.insert_node_and_edge(prevId, childId, chainNodes[i + 1], e);
        prevId = childId;
    }

    po.goalNodeId           = prevId;
    po.bestNodeId           = prevId;
    po.bestNodeIdCostToGoal = 0;
    po.success              = true;
    po.pathCost             = tree.nodes().at(prevId).cost_;

    MRPT_LOG_DEBUG_STREAM(
        "TPS_Astar_Bidir: solved. iters="
        << nIter << " mu=" << bestMu << " edges=" << chainEdges.size());

    return po;
    MRPT_END
}
