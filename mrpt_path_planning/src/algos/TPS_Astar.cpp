/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/algos/render_tree.h>
#include <mpp/algos/tp_obstacles_single_path.h>
#include <mpp/algos/transform_pc_square_clipping.h>
#include <mpp/algos/within_bbox.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/COpenGLScene.h>

#include <iostream>
#include <unordered_set>

IMPLEMENTS_MRPT_OBJECT(TPS_Astar, Planner, mpp)

using namespace mpp;

mrpt::containers::yaml TPS_Astar_Parameters::as_yaml()
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    // TODO: Add comments
    MCP_SAVE(c, SE2_metricAngleWeight);
    MCP_SAVE(c, pathInterpolatedSegments);
    MCP_SAVE(c, saveDebugVisualizationDecimation);
    MCP_SAVE(c, debugVisualizationShowEdgeCosts);
    MCP_SAVE(c, grid_resolution_xy);
    MCP_SAVE(c, heuristic_heading_weight);
    MCP_SAVE(c, max_ptg_trajectories_to_explore);
    MCP_SAVE(c, max_ptg_speeds_to_explore);
    MCP_SAVE_DEG(c, grid_resolution_yaw);
    MCP_SAVE(c, maximumComputationTime);

    c["ptg_sample_timestamps"] = mrpt::containers::yaml::Sequence();
    for (const auto& v : ptg_sample_timestamps)
        c["ptg_sample_timestamps"].asSequence().push_back(v);

    return c;
}

void TPS_Astar_Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_REQ(c, grid_resolution_xy);
    MCP_LOAD_REQ_DEG(c, grid_resolution_yaw);

    MCP_LOAD_REQ(c, SE2_metricAngleWeight);
    MCP_LOAD_REQ(c, max_ptg_trajectories_to_explore);
    MCP_LOAD_REQ(c, max_ptg_speeds_to_explore);

    ASSERT_(
        c.has("ptg_sample_timestamps") &&
        c["ptg_sample_timestamps"].isSequence());
    ptg_sample_timestamps = c["ptg_sample_timestamps"].toStdVector<double>();

    MCP_LOAD_OPT(c, pathInterpolatedSegments);
    MCP_LOAD_OPT(c, saveDebugVisualizationDecimation);
    MCP_LOAD_OPT(c, debugVisualizationShowEdgeCosts);
    MCP_LOAD_OPT(c, heuristic_heading_weight);

    MCP_LOAD_OPT(c, maximumComputationTime);
}

TPS_Astar_Parameters TPS_Astar_Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    TPS_Astar_Parameters p;
    p.load_from_yaml(c);
    return p;
}

TPS_Astar::TPS_Astar() : mrpt::system::COutputLogger("TPS_Astar")
{
    profiler_().setName("TPS_Astar");
}

PlannerOutput TPS_Astar::plan(const PlannerInput& in)
{
    MRPT_START
    mrpt::system::CTimeLoggerEntry tleg(profiler_(), "plan");

    const double planInitTime = mrpt::Clock::nowDouble();

    // Sanity checks on inputs:
    ASSERT_(in.ptgs.initialized());
    ASSERT_(in.worldBboxMin != in.worldBboxMax);
    ASSERT_(within_bbox(in.stateStart.pose, in.worldBboxMax, in.worldBboxMin));

    ASSERT_(!in.stateGoal.state.isEmpty());
    if (in.stateGoal.state.isPoint())
        ASSERT_(within_bbox(
            in.stateGoal.state.point(), in.worldBboxMax, in.worldBboxMin));
    else if (in.stateGoal.state.isPose())
        ASSERT_(within_bbox(
            in.stateGoal.state.pose(), in.worldBboxMax, in.worldBboxMin));

    MRPT_LOG_DEBUG_STREAM("Starting planning.");
    MRPT_LOG_DEBUG_STREAM("from " << in.stateStart.asString());
    MRPT_LOG_DEBUG_STREAM("to " << in.stateGoal.asString());
    MRPT_LOG_DEBUG_STREAM("Obstacle sources: " << in.obstacles.size());
    MRPT_LOG_DEBUG_STREAM("Cost evaluators: " << costEvaluators_.size());

    PlannerOutput po;
    po.originalInput = in;

    auto& tree = po.motionTree;  // shortcut

    // clipping dist for all ptgs:
    double MAX_XY_DIST = 0;
    for (const auto& ptg : in.ptgs.ptgs)
        mrpt::keep_max(MAX_XY_DIST, ptg->getRefDistance());
    ASSERT_(MAX_XY_DIST > 0);

    // obstacles (TODO: dynamic over future time?):
    std::vector<mrpt::maps::CPointsMap::Ptr> obstaclePoints;
    for (const auto& os : in.obstacles)
        if (os) obstaclePoints.emplace_back(os->obstacles());

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    grid_.setSize(
        in.worldBboxMin.x, in.worldBboxMax.x,  // x
        in.worldBboxMin.y, in.worldBboxMax.y,  // y
        params_.grid_resolution_xy, params_.grid_resolution_yaw,  // res
        in.worldBboxMin.phi, in.worldBboxMax.phi  // phi / yaw
    );

    // ----------------------------------------
    //
    // A* algorithm
    //
    // ----------------------------------------
    // Open set is keyed by fScore (estimated cost to goal).
    std::multimap<distance_t, NodePtr> openSet;
    mrpt::graphs::TNodeID              nextFreeId = 0;

    // openSet <- startNode
    {
        auto& n = getOrCreateNodeByPose(in.stateStart, nextFreeId);
        n.state = in.stateStart;

        //   X_T ← {X_0 }    # Tree nodes (state space)
        // ------------------------------------------------------------------
        tree.root = n.id.value();
        tree.insert_root_node(tree.root, n.state);

        n.gScore           = 0;
        n.fScore           = heuristic(n.state, in.stateGoal);
        n.pendingInOpenSet = true;

        openSet.insert({n.fScore, &n});
    }

    // Define goal node ID:
    // Use a pointer so we can replace it with a pointer to a different node as
    // needed.
    auto* nodeGoal =
        &getOrCreateNodeByPose(in.stateGoal.asSE2KinState(), nextFreeId);
    po.goalNodeId = nodeGoal->id.value();

    // Goal cell indices:
    const auto goalCellIndices =
        in.stateGoal.state.isPoint()
            ? nodeGridCoords(in.stateGoal.state.point())
            : nodeGridCoords(in.stateGoal.state.pose());

    // goal speed=0
    MRPT_TODO("Actually check user input on desired speed at goal");
    nodes_with_desired_speed_t nodesWithDesiredSpeed;
    nodesWithDesiredSpeed[goalCellIndices] = 0;

    unsigned int nIter = 0;

    double tLastCallback = planInitTime;

    while (!openSet.empty())
    {
        mrpt::system::CTimeLoggerEntry tle(profiler_(), "plan.iter");

        nIter++;  // just for debugging purposes

        // node with the lowest fScore:
        Node& current = *openSet.begin()->second.ptr;

        // current==goal?
        // we must check the state to be on the same lattice cell to check
        // for a match of the current SE(2) pose against the goal state,
        // which may be either a SE(2) pose or a R2 point:
        if (const auto curNodeGridIdx = nodeGridCoords(current.state.pose);
            curNodeGridIdx.sameLocation(goalCellIndices))
        {
            // Path found:

            // Redefine the goal cell index to the current one, for the case
            // of goal not having a desired heading, in which case we formerly
            // defined a temporary/instrumental goal cell with phi=0, but we now
            // want the actual, exact final cell index:
            if (in.stateGoal.state.isPoint())
            {
                // Correct current node location due to grid discretization,
                // a latter refine_trajectory() stage will correct PTG path
                // params if needed:
                const auto& goalPt = in.stateGoal.state.point();
                // Update in our internal A* node:
                current.state.pose.x = goalPt.x;
                current.state.pose.y = goalPt.y;

                MRPT_LOG_DEBUG_STREAM(
                    "Path found to R(2) goal point: redefining goal from "
                    << in.stateGoal.state.asString() << " ==> "
                    << current.state.asString());

                nodeGoal                = &current;
                po.goalNodeId           = nodeGoal->id.value();
                po.bestNodeId           = po.goalNodeId;
                po.bestNodeIdCostToGoal = 0;
            }
            else
            {
                // Correct current node location due to grid discretization,
                // a latter refine_trajectory() stage will correct PTG path
                // params if needed:
                const auto& goalPose = in.stateGoal.state.pose();
                // Update in our internal A* node:
                current.state.pose = goalPose;

                po.bestNodeIdCostToGoal = 0;
            }
            // Update in the tree node also:
            tree.node_state(*current.id).pose = current.state.pose;

            break;
        }

        // remove it from open set:
        current.pendingInOpenSet = false;
        current.visited          = true;
        openSet.erase(openSet.begin());

        // for each neighbor of current:
        const auto neighbors = find_feasible_paths_to_neighbors(
            current, in.ptgs, in.stateGoal, obstaclePoints, MAX_XY_DIST,
            nodesWithDesiredSpeed);

#if 0
        std::cout << " cur : " << nodeGridCoords(current.state.pose).asString()
                  << "\n";
        std::cout << " goal: " << nodeGridCoords(nodeGoal.state.pose).asString()
                  << "\n";
#endif

        for (const auto& edge : neighbors)
        {
            // d(current,neighbor) is the weight of the edge from current to
            // neighbor

            // tentative_gScore is the distance from start to the neighbor
            // through current

            // tentative_gScore := gScore[current] + d(current, neighbor)

            auto& ptg = *in.ptgs.ptgs.at(edge.ptgIndex.value());

            ptg.updateNavDynamicState(edge.ptgDynState.value());
            if (auto* ptgTrim = dynamic_cast<ptg::SpeedTrimmablePTG*>(&ptg);
                ptgTrim)
                ptgTrim->trimmableSpeed_ = edge.ptgTrimmableSpeed;

            const uint32_t ptg_step        = edge.relTrgStep.value();
            const auto&    reconstrRelPose = edge.relReconstrPose;

            // new tentative node pose & velocity:
            const auto q_i = current.state.pose + reconstrRelPose;
            const auto relTwist =
                ptg.getPathTwist(edge.ptgTrajIndex.value(), ptg_step);

            SE2_KinState x_i;
            x_i.pose = q_i;
            // relTwist is relative to the *parent* (srcNode) frame:
            (x_i.vel = relTwist).rotate(current.state.pose.phi);

            // Out of world bounding box? It might happen due to lattice
            // rounding differences between the checks in
            // find_feasible_paths_to_neighbors() and the actual PTG path
            // segments.
            if (q_i.x < in.worldBboxMin.x || q_i.y < in.worldBboxMin.y ||
                q_i.phi < in.worldBboxMin.phi)
                continue;
            if (q_i.x > in.worldBboxMax.x || q_i.y > in.worldBboxMax.y ||
                q_i.phi > in.worldBboxMax.phi)
                continue;

            // Get or create node:
            auto& neighborNode = getOrCreateNodeByPose(x_i, nextFreeId);

            // Skip if already visited:
            if (neighborNode.visited) continue;

            // Build a tentative new edge data structure.
            // It will be used to be inserted in the graph, if accepted, and
            // in any case, to evaluate the edge cost.
            MoveEdgeSE2_TPS newEdge;

            newEdge.parentId     = current.id.value();
            newEdge.ptgDist      = edge.ptgDist;
            newEdge.ptgIndex     = edge.ptgIndex.value();
            newEdge.ptgPathIndex = edge.ptgTrajIndex.value();

            newEdge.ptgTrimmableSpeed    = edge.ptgTrimmableSpeed;
            newEdge.ptgFinalGoalRelSpeed = 0;
            newEdge.ptgFinalRelativeGoal =
                in.stateGoal.asSE2KinState().pose - current.state.pose;

            newEdge.stateFrom = current.state;
            newEdge.stateTo   = x_i;

            // interpolated path:
            edge_interpolated_path(
                newEdge, in.ptgs, reconstrRelPose, ptg_step,
                params_.pathInterpolatedSegments);

            // Let's compute its cost:
            newEdge.cost = cost_path_segment(newEdge);
            ASSERT_GT_(newEdge.cost, .0);

            const cost_t tentative_gScore = current.gScore + newEdge.cost;

            // Better path? If it is not, go on with the next edge:
            if (tentative_gScore >= neighborNode.gScore) continue;

            // YES: accept this new edge
            // --------------------------------
            const bool hasToRewire = neighborNode.cameFrom.has_value();

            // This path to neighbor is better than any previous one,
            // overwrite it:
            neighborNode.cameFrom = &current;
            neighborNode.gScore   = tentative_gScore;

            // fScore[neighbor] := tentative_gScore + h(neighbor)
            const cost_t costToGoal =
                heuristic(neighborNode.state, in.stateGoal);
            neighborNode.fScore = tentative_gScore + costToGoal;

            if (!neighborNode.pendingInOpenSet)
            {
                neighborNode.pendingInOpenSet = true;
                openSet.insert({neighborNode.fScore, &neighborNode});
            }

            // Overwrite state with new one:
            neighborNode.state = x_i;

            // Delete old edge, if any:
            if (hasToRewire)
            {
                // do rewire:
                tree.rewire_node_parent(neighborNode.id.value(), newEdge);
            }
            else
            {
                // Add edge to tree:
                tree.insert_node_and_edge(
                    newEdge.parentId, neighborNode.id.value(),
                    neighborNode.state, newEdge);
            }

            // Keep an updated pointer to the best, so-far, node (under the
            // heuristic criterion):
            // Note that even if costToGoal==0, we may still need to keep
            // iterating A*, since a better (shorter) path might be still be
            // found.
            if (costToGoal < po.bestNodeIdCostToGoal)
            {
                po.bestNodeIdCostToGoal = costToGoal;
                po.bestNodeId           = neighborNode.id.value();
            }

        }  // end for each edge to neighbor

        MRPT_LOG_DEBUG_FMT(
            "iter: %4u %65s neighbors=%3u fS=%.02f gS=%.02f |openSet|=%u",
            nIter, current.state.asString().c_str(),
            static_cast<unsigned int>(neighbors.size()), current.fScore,
            current.gScore, static_cast<unsigned int>(openSet.size()));

        // Debug log files:
        if (params_.saveDebugVisualizationDecimation > 0 &&
            (nIter % params_.saveDebugVisualizationDecimation) == 0)
        {
            RenderOptions ro;
            ro.highlight_path_to_node_id = current.id.value();
            ro.showEdgeCosts = params_.debugVisualizationShowEdgeCosts;
            mrpt::opengl::COpenGLScene scene;
            scene.insert(render_tree(tree, in, ro));
            scene.saveToFile(mrpt::format("debug_astar_%05u.3Dscene", nIter));
        }

        // Time-based events:
        const double tNow = mrpt::Clock::nowDouble();

        // timeout?
        if ((tNow - planInitTime) > params_.maximumComputationTime)
        {
            // timeout
            MRPT_LOG_DEBUG("Timeout.");
            break;
        }

        // Periodically report what is the so-far best path:
        if (progressCallback_ &&
            (tNow - tLastCallback) > progressCallbackCallPeriod_)
        {
            tLastCallback = tNow;

            const auto [foundPath, pathEdges] =
                tree.backtrack_path(*po.bestNodeId);

            // call user callback:
            ProgressCallbackData pcd;
            pcd.bestCostFromStart = tree.nodes().at(*po.bestNodeId).cost_;
            pcd.bestCostToGoal    = po.bestNodeIdCostToGoal;
            pcd.bestFinalNode     = po.bestNodeId;
            pcd.bestPath          = std::move(pathEdges);
            pcd.costEvaluators    = &costEvaluators_;
            pcd.originalPlanInput = &in;
            pcd.tree              = &tree;

            progressCallback_(pcd);
        }

    }  // end while openSet!=empty

    // A* ended, now collect the result:
    // ----------------------------------------
#if 0  // debug: dump tree
    tree.visitBreadthFirst(
        tree.root, [](const TNodeID parent, const auto& edgeToChild,
                      const size_t depthLevel) {
            const MoveEdgeSE2_TPS& e = edgeToChild.data;

            std::cout << "tree level #" << depthLevel << ": parent=" << parent
                      << " edgeToChild: " << e.asString() << "\n";
        });
#endif

    po.success = po.goalNodeId == po.bestNodeId;
    if (po.bestNodeId) po.pathCost = tree.nodes().at(*po.bestNodeId).cost_;

    po.computationTime = mrpt::Clock::nowDouble() - planInitTime;

    return po;
    MRPT_END
}

cost_t TPS_Astar::default_heuristic_SE2(
    const SE2_KinState& from, const mrpt::math::TPose2D& goal) const
{
    mpp::PoseDistanceMetric_Lie<mpp::SE2_KinState> metric(
        params_.SE2_metricAngleWeight);

    // Distance in SE(2):
    const double distSE2 = metric.distance(from.pose, goal);

    // Favor heading towards the target, if we are far away:
    const auto   relPose = goal - from.pose;
    const double distHeading =
        (relPose.norm() < 0.1)
            ? 0.0
            : std::abs(mrpt::math::angDistance(
                  std::atan2(relPose.y, relPose.x), from.pose.phi));

    return distSE2 + params_.heuristic_heading_weight * distHeading;
}

cost_t TPS_Astar::default_heuristic_R2(
    const SE2_KinState& from, const mrpt::math::TPoint2D& goal) const
{
    // Distance in R^2:
    const double distR2 = (from.pose.translation() - goal).norm();

    // Favor heading towards the target, if we are far away:
    const auto   relPose = goal - from.pose;
    const double distHeading =
        (relPose.norm() < 0.1)
            ? 0.0
            : std::abs(mrpt::math::angDistance(
                  std::atan2(relPose.y, relPose.x), from.pose.phi));

    return distR2 + params_.heuristic_heading_weight * distHeading;
}

cost_t TPS_Astar::default_heuristic(
    const SE2_KinState& from, const SE2orR2_KinState& goal) const
{
    if (goal.state.isPoint())
        return default_heuristic_R2(from, goal.state.point());
    else if (goal.state.isPose())
        return default_heuristic_SE2(from, goal.state.pose());
    else
        THROW_EXCEPTION("Goal of unknown type?");
}

TPS_Astar::list_paths_to_neighbors_t
    TPS_Astar::find_feasible_paths_to_neighbors(
        const TPS_Astar::Node& from, const TrajectoriesAndRobotShape& trs,
        const SE2orR2_KinState&                         goalState,
        const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
        double                            MAX_XY_OBSTACLES_CLIPPING_DIST,
        const nodes_with_desired_speed_t& nodesWithSpeed)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "find_feasible");

    // const auto iFromCoords = nodeGridCoords(from.state.pose);

    const NodeCoords iGoalCoords = goalState.state.isPoint()
                                       ? nodeGridCoords(goalState.state.point())
                                       : nodeGridCoords(goalState.state.pose());

    const auto relGoal = goalState.asSE2KinState().pose - from.state.pose;

    const double halfCell = grid_.getResolutionXY() * 0.5;

    // local obstacles as seen from this "from" pose:
    const auto localObstacles = cached_local_obstacles(
        from.state.pose, globalObstacles, MAX_XY_OBSTACLES_CLIPPING_DIST);

    // If two PTGs reach the same cell, keep the shortest/best:
    std::map<absolute_cell_index_t, path_to_neighbor_t> bestPaths;

    size_t totalConsidered = 0, totalCollided = 0;

    // For each PTG:
    for (size_t ptgIdx = 0; ptgIdx < trs.ptgs.size(); ptgIdx++)
    {
        mrpt::system::CTimeLoggerEntry tleL1(
            profiler_(), "find_feasible.loop1");

        auto& ptg = trs.ptgs.at(ptgIdx);
        ASSERT_(ptg->isInitialized());

        // This will be !=null if the PTG supports trimmable speeds:
        auto ptgTrimmable =
            std::dynamic_pointer_cast<ptg::SpeedTrimmablePTG>(ptg);

        const duration_seconds_t ptg_dt = ptg->getPathStepDuration();

        // Update PTG dynamics:
        {
            ptg_t::TNavDynamicState ds;
            (ds.curVelLocal = from.state.vel).rotate(-from.state.pose.phi);

            ds.relTarget = relGoal;

            if (const auto it = nodesWithSpeed.find(iGoalCoords);
                it != nodesWithSpeed.end())
            {
                MRPT_TODO("Speed zone filter here too?");
                ds.targetRelSpeed = it->second;
            }
            else
            {
                MRPT_TODO("Support case of final goal speed!=0 ?");
                ds.targetRelSpeed = 0;
            }

            mrpt::system::CTimeLoggerEntry tle3(
                profiler_(), "find_feasible.ptgUpdateDyn");

            ptg->updateNavDynamicState(ds);

            tle3.stop();
        }

        // explore a subset of all trajectories only:
        std::set<trajectory_index_t> trajIdxsToConsider;
        std::vector<TPS_point>       tpsPointsToConsider;
        std::set<size_t> targetTpsPointIndx;  // if reachable with ptg

        ASSERT_(params_.max_ptg_trajectories_to_explore >= 2);
        for (size_t i = 0; i < params_.max_ptg_trajectories_to_explore; i++)
        {
            trajectory_index_t trjIdx = mrpt::round(
                i * (ptg->getPathCount() - 1) /
                (params_.max_ptg_trajectories_to_explore - 1));
            trajIdxsToConsider.insert(trjIdx);
        }

        // Build possible distances for each path:
        std::vector<normalized_speed_t> speedsToConsider;
        if (ptgTrimmable)
        {
            // N=1 ==>  [1.0]
            // N=2 ==>  [0.5, 1.0]
            // N=3 ==>  [0.33, 0.66, 1.0]
            // ....

            ASSERT_(params_.max_ptg_speeds_to_explore >= 1);
            normalized_speed_t speedStep =
                1.0 / params_.max_ptg_speeds_to_explore;
            for (normalized_speed_t s = speedStep; s < 1.001; s += speedStep)
                speedsToConsider.push_back(s);
        }
        else
        {
            // No speed-trimmable PTG:
            speedsToConsider.push_back(1.0);
        }

        // make sure of including the trajectory towards the target, if we
        // are close enough, plus its immediate neighboring paths:
        {
            int                   relTrg_k       = 0;
            normalized_distance_t relTrg_d       = 0;
            const double          queryTolerance = params_.grid_resolution_xy;
            if (ptg->inverseMap_WS2TP(
                    relGoal.x, relGoal.y, relTrg_k, relTrg_d, queryTolerance))
            {
                ptg_step_t relTrg_step = 0;
                if (ptg->getPathStepForDist(relTrg_k, relTrg_d, relTrg_step))
                {
                    // Add direct path to target, and keep a copy of its value:
                    for (auto speed : speedsToConsider)
                    {
                        const auto trgTps =
                            TPS_point(relTrg_k, relTrg_step, speed);

                        tpsPointsToConsider.emplace_back(trgTps);

                        // save its index:
                        targetTpsPointIndx.insert(
                            tpsPointsToConsider.size() - 1);
                    }
                }

                // and also, in general, the path:
                trajIdxsToConsider.insert(relTrg_k);
            }
        }

        for (const auto speed : speedsToConsider)
        {
            for (const auto trjIdx : trajIdxsToConsider)
            {
                for (duration_seconds_t t : params_.ptg_sample_timestamps)
                {
                    ptg_step_t trjStep = mrpt::round(t / ptg_dt);

                    // skip if this PTG path ends earlier than the specified
                    // timestamp:
                    const auto maxSteps = ptg->getPathStepCount(trjIdx);

                    ASSERT_(maxSteps >= 1);
                    if (trjStep >= maxSteps) continue;

                    tpsPointsToConsider.emplace_back(trjIdx, trjStep, speed);
                }
            }
        }

        tleL1.stop();

        mrpt::system::CTimeLoggerEntry tleL2(
            profiler_(), "find_feasible.loop2");

        std::unordered_set<NodeCoords, NodeCoordsHash> goalNodeCoords;

        // now, check which ones of those paths are not blocked by
        // obstacles:
        for (size_t tpsPtIdx = 0; tpsPtIdx < tpsPointsToConsider.size();
             tpsPtIdx++)
        {
            const auto& tpsPt = tpsPointsToConsider[tpsPtIdx];

            totalConsidered++;

            // solution is a no-motion: skip.
            if (tpsPt.step == 0) continue;

            // Update speed modulation in the PTG:
            if (ptgTrimmable && tpsPt.speed != ptgTrimmable->trimmableSpeed_)
                ptgTrimmable->trimmableSpeed_ = tpsPt.speed;

            // Reconstruct the actual global pose:
            const auto relTrgDist      = ptg->getPathDist(tpsPt.k, tpsPt.step);
            const auto relReconstrPose = ptg->getPathPose(tpsPt.k, tpsPt.step);
            const auto absPose         = from.state.pose + relReconstrPose;

            // out of lattice limits?
            if (absPose.x < grid_.getXMin() || absPose.y < grid_.getYMin() ||
                absPose.phi < grid_.getPhiMin())
                continue;
            if (absPose.x > grid_.getXMax() - halfCell ||
                absPose.y > grid_.getYMax() - halfCell ||
                absPose.phi > grid_.getPhiMax())
                continue;

            const NodeCoords nc = nodeGridCoords(absPose);

            mrpt::system::CTimeLoggerEntry tleObs(
                profiler_(), "find_feasible.tp_obstacles_single");

            // check for collisions:
            const distance_t freeDistance =
                tp_obstacles_single_path(tpsPt.k, *localObstacles, *ptg);

            tleObs.stop();

            if (relTrgDist >= freeDistance)
            {
                // we would need to move farther away than what is possible
                // without colliding: discard this trajectory.
                totalCollided++;
                continue;
            }

            // It is a collision-free path.

            // Is this a direct path to goal?
            // If it is, do not try to consider other paths that end up
            // in the same lattice cell, even if shorter, since we prefer
            // to reach the goal pose as exactly as possible:
            if (targetTpsPointIndx.count(tpsPtIdx) != 0)
            {  // add it:
                goalNodeCoords.insert(nc);
            }
            else
            {
                // are we reaching a goal cell and this is not a
                // straight-to-goal path?
                if (goalNodeCoords.count(nc) != 0)
                {
                    // skip it
                    continue;
                }
            }

            // ok, it's a good potential path, add it.
            // It will be later on scored by the A* algo.

            auto& path = bestPaths[nodeCoordsToAbsIndex(nc)];

            // Ok, it's a valid new neighbor with this PTG.
            // Is it shorter with this PTG than with others?
            if (relTrgDist < path.ptgDist)
            {
                path.ptgDist            = relTrgDist;
                path.ptgIndex           = ptgIdx;
                path.ptgTrajIndex       = tpsPt.k;
                path.relReconstrPose    = relReconstrPose;
                path.relTrgStep         = tpsPt.step;
                path.neighborNodeCoords = nc;
                path.ptgDynState        = ptg->getCurrentNavDynamicState();
            }
        }

        tleL2.stop();

    }  // end for each PTG

    mrpt::system::CTimeLoggerEntry tleF(profiler_(), "find_feasible.finalFill");

    // Fill "neighbors" from valid "bestPaths":
    list_paths_to_neighbors_t neighbors;

    for (const auto& kv : bestPaths)
    {
        const auto& path = kv.second;
        if (!path.ptgIndex.has_value()) continue;  // skip

        neighbors.emplace_back(path);
    }

#if 0
    MRPT_LOG_DEBUG_STREAM(
        "find_feasible() for p="
        << from.state.pose << " => " << totalConsidered << "/" << totalCollided
        << "/" << neighbors.size() << " considered/collided/accepted.");
#endif

    tleF.stop();

    return neighbors;
}

mrpt::maps::CPointsMap::Ptr TPS_Astar::cached_local_obstacles(
    const mrpt::math::TPose2D&                      queryPose,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
    double                                          MAX_PTG_XY_DIST)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "cached_local_obstacles");

    MRPT_TODO("Impl actual cache");

    auto outObs = mrpt::maps::CSimplePointsMap::Create();

    for (const auto& obs : globalObstacles)
    {
        ASSERT_(obs);
        transform_pc_square_clipping(
            *obs, mrpt::poses::CPose2D(queryPose), MAX_PTG_XY_DIST, *outObs);
    }

    return outObs;
}
