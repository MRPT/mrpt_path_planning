/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/algos/render_tree.h>
#include <selfdriving/algos/tp_obstacles_single_path.h>
#include <selfdriving/algos/transform_pc_square_clipping.h>
#include <selfdriving/algos/within_bbox.h>
#include <selfdriving/data/MotionPrimitivesTree.h>

#include <iostream>

IMPLEMENTS_MRPT_OBJECT(TPS_Astar, Planner, selfdriving)

using namespace selfdriving;

mrpt::containers::yaml TPS_Astar_Parameters::as_yaml()
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    // TODO: Add comments
    MCP_SAVE(c, SE2_metricAngleWeight);
    MCP_SAVE(c, pathInterpolatedSegments);
    MCP_SAVE(c, saveDebugVisualizationDecimation);
    MCP_SAVE(c, grid_resolution_xy);
    MCP_SAVE_DEG(c, grid_resolution_yaw);

    return c;
}

void TPS_Astar_Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_REQ(c, grid_resolution_xy);
    MCP_LOAD_REQ_DEG(c, grid_resolution_yaw);

    MCP_LOAD_REQ(c, SE2_metricAngleWeight);
    MCP_LOAD_OPT(c, pathInterpolatedSegments);
    MCP_LOAD_OPT(c, saveDebugVisualizationDecimation);
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
    profiler_.setName("TPS_Astar");
}

PlannerOutput TPS_Astar::plan(const PlannerInput& in)
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
    std::multimap<distance_t, NodePtr> openSet;
    mrpt::graphs::TNodeID              nextFreeId = 0;

    // openSet <- startNode
    {
        auto& n = getOrCreateNodeByPose(in.stateStart, tree, nextFreeId);
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
    auto& nodeGoal = getOrCreateNodeByPose(in.stateGoal, tree, nextFreeId);
    po.goalNodeId  = nodeGoal.id.value();

    // Insert a dummy edge between root -> goal, just to allow new node IDs
    // to be generated in sequence:
    {
        MoveEdgeSE2_TPS dummyEdge;
        dummyEdge.cost      = std::numeric_limits<cost_t>::max();
        dummyEdge.parentId  = tree.root;
        dummyEdge.stateFrom = in.stateStart;
        dummyEdge.stateTo   = in.stateGoal;
        tree.insert_node_and_edge(
            tree.root, nodeGoal.id.value(), in.stateGoal, dummyEdge);
    }

    nodes_with_exact_coordinates_t nodesWithExactCoords;
    nodesWithExactCoords[nodeGridCoords(in.stateGoal.pose)] = in.stateGoal;

    // goal speed=0
    nodes_with_desired_speed_t nodesWithDesiredSpeed;
    nodesWithDesiredSpeed[nodeGridCoords(in.stateGoal.pose)] = 0;

    unsigned int nIter = 0;

    while (!openSet.empty())
    {
        nIter++;  // just for debugging purposes

        // node with the lowest fScore:
        Node& current = *openSet.begin()->second.ptr;

        // current==goal?
        if (current.id.value() == nodeGoal.id.value())
        {
            // Path found:
            break;
        }

        // remove it from open set:
        current.pendingInOpenSet = false;
        current.visited          = true;
        openSet.erase(openSet.begin());

        // for each neighbor of current:
        const auto neighbors = find_feasible_paths_to_neighbors(
            current, in.ptgs, in.stateGoal, nodesWithExactCoords,
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

            const auto& ptg = *in.ptgs.ptgs.at(edge.ptgIndex.value());

            uint32_t ptg_step;
            bool     stepOk = ptg.getPathStepForDist(
                edge.ptgTrajIndex.value(), edge.ptgDist, ptg_step);
            ASSERT_(stepOk);

            const auto reconstrRelPose =
                ptg.getPathPose(edge.ptgTrajIndex.value(), ptg_step);

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
            auto& neighborNode = getOrCreateNodeByPose(x_i, tree, nextFreeId);

            // Skip if already visited:
            if (neighborNode.visited) continue;

            // Build a tentative new edge data structure.
            // It will be used to be inserted in the graph, if accepted, and in
            // any case, to evaluate the edge cost.
            MoveEdgeSE2_TPS newEdge;

            newEdge.parentId     = current.id.value();
            newEdge.ptgDist      = edge.ptgDist;
            newEdge.ptgIndex     = edge.ptgIndex.value();
            newEdge.ptgPathIndex = edge.ptgTrajIndex.value();
            MRPT_TODO("targetRelSpeed?");
            // tentativeEdge.targetRelSpeed = ds.targetRelSpeed;
            newEdge.stateFrom = current.state;
            newEdge.stateTo   = x_i;
            // interpolated path:
            if (const auto nSeg = params_.pathInterpolatedSegments; nSeg > 0)
            {
                auto& ip = newEdge.interpolatedPath.emplace();
                ip.emplace_back(0, 0, 0);  // fixed
                // interpolated:
                for (size_t i = 0; i < nSeg; i++)
                {
                    const auto iStep = ((i + 1) * ptg_step) / (nSeg + 2);
                    ip.emplace_back(
                        ptg.getPathPose(newEdge.ptgPathIndex, iStep));
                }
                ip.emplace_back(reconstrRelPose);  // already known
            }

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
            neighborNode.fScore = tentative_gScore +
                                  heuristic(neighborNode.state, nodeGoal.state);

            if (!neighborNode.pendingInOpenSet)
            {
                neighborNode.pendingInOpenSet = true;
                openSet.insert({neighborNode.fScore, &neighborNode});
            }

            // Overwrite state with new one:
            neighborNode.state = x_i;

            // Delete old edge, if any:
            if (hasToRewire)
            { tree.rewire_node_parent(neighborNode.id.value(), newEdge); }
            else
            {
                // Add edge to tree:
                tree.insert_node_and_edge(
                    newEdge.parentId, neighborNode.id.value(),
                    neighborNode.state, newEdge);
            }

        }  // end for each edge to neighbor

        MRPT_LOG_DEBUG_FMT(
            "iter: %4u %35s neighbors=%3u fS=%.02f gS=%.02f", nIter,
            current.state.asString().c_str(),
            static_cast<unsigned int>(neighbors.size()), current.fScore,
            current.gScore);

        // Debug log files:
        if (params_.saveDebugVisualizationDecimation > 0 &&
            (nIter % params_.saveDebugVisualizationDecimation) == 0)
        {
            RenderOptions ro;
            ro.highlight_path_to_node_id = current.id.value();
            mrpt::opengl::COpenGLScene scene;
            scene.insert(render_tree(tree, in, ro));
            scene.saveToFile(mrpt::format("debug_astar_%05u.3Dscene", nIter));
        }

    }  // end while openSet!=empty

    // A* ended, now collect the result:
    // ----------------------------------------
    const auto foundPath      = tree.backtrack_path(nodeGoal.id.value());
    bool       foundPathValid = true;

    for (const auto& step : foundPath)
    {
        if (step.cost_ == std::numeric_limits<distance_t>::max())
        {
            foundPathValid = false;
            break;
        }
    }
    po.success = foundPathValid;

    po.pathCost = tree.nodes().at(nodeGoal.id.value()).cost_;

    return po;
    MRPT_END
}

distance_t TPS_Astar::default_heuristic(
    const SE2_KinState& from, const SE2_KinState& goal) const
{
    selfdriving::PoseDistanceMetric_Lie<selfdriving::SE2_KinState> metric(
        params_.SE2_metricAngleWeight);

    // Distance in SE(2):
    const double distSE2 = metric.distance(from.pose, goal.pose);

    // Favor heading towards the target, if we are far away:
    const auto   relPose = goal.pose - from.pose;
    const double distHeading =
        (relPose.norm() < 2.0)
            ? 0.0
            : std::abs(mrpt::math::angDistance(
                  std::atan2(relPose.y, relPose.x), goal.pose.phi));

    return distSE2 + distHeading;
}

TPS_Astar::list_paths_to_neighbors_t
    TPS_Astar::find_feasible_paths_to_neighbors(
        const TPS_Astar::Node& from, const TrajectoriesAndRobotShape& trs,
        const SE2_KinState&                   goalState,
        const nodes_with_exact_coordinates_t& nodesWithExactCoords,
        const nodes_with_desired_speed_t&     nodesWithSpeed)
{
    const auto iFromCoords = nodeGridCoords(from.state.pose);
    const auto iGoalCoords = nodeGridCoords(goalState.pose);

    const size_t MAX_PTG_TRAJ_TO_EXPLORE            = 20;
    const double PTG_NORM_DIST_GRANULARITY_SAMPLING = 0.2;

    const auto relGoal = goalState.pose - from.state.pose;

    // If two PTGs reach the same cell, keep the shortest/best:
    std::map<absolute_cell_index_t, path_to_neighbor_t> bestPaths;

    // For each PTG:
    for (size_t ptgIdx = 0; ptgIdx < trs.ptgs.size(); ptgIdx++)
    {
        auto& ptg = trs.ptgs.at(ptgIdx);
        ASSERT_(ptg->isInitialized());

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
                ds.targetRelSpeed = 1.0;
            }

            ptg->updateNavDynamicState(ds);
        }

        // explore a subset of all trajectories only:
        std::set<trajectory_index_t> trajIdxsToConsider;
        std::vector<TPS_point>       tpsPointsToConsider;

        for (size_t i = 0; i < MAX_PTG_TRAJ_TO_EXPLORE; i++)
        {
            trajectory_index_t trjIdx = mrpt::round(
                i * (ptg->getPathCount() - 1) / (MAX_PTG_TRAJ_TO_EXPLORE - 1));
            trajIdxsToConsider.insert(trjIdx);
        }

        // make sure of including the trajectory towards the target, if we are
        // close enough, plus its immediate neighboring paths:
        {
            int                   relTrg_k       = 0;
            normalized_distance_t relTrg_d       = 0;
            const double          queryTolerance = params_.grid_resolution_xy;
            if (ptg->inverseMap_WS2TP(
                    relGoal.x, relGoal.y, relTrg_k, relTrg_d, queryTolerance))
            {
                // Add direct path to target:
                tpsPointsToConsider.emplace_back(relTrg_k, relTrg_d);
                // and also, in general, the path:
                trajIdxsToConsider.insert(relTrg_k);
            }
        }

        // Build possible distances for each path:
        for (const auto trjIdx : trajIdxsToConsider)
        {
            for (normalized_distance_t d = PTG_NORM_DIST_GRANULARITY_SAMPLING;
                 d < 0.999; d += PTG_NORM_DIST_GRANULARITY_SAMPLING)
            {  //
                tpsPointsToConsider.emplace_back(trjIdx, d);
            }
        }

        // now, check which ones of those paths are not blocked by obstacles:
        for (const auto& tpsPt : tpsPointsToConsider)
        {
            // check collisions:
            MRPT_TODO("tps-obstacles");

            bool collision = false;

            if (collision) continue;

            // ok, it's a good potential path:
            // (it will be later on scored by the A* algo)

            // Reconstruct the actual global pose:
            distance_t dist = tpsPt.d * ptg->getRefDistance();

            uint32_t relTrgStep;
            bool stepOk = ptg->getPathStepForDist(tpsPt.k, dist, relTrgStep);
            if (!stepOk) continue;

            // solution is a no-motion: skip.
            if (relTrgStep == 0) continue;

            const auto relReconstrPose = ptg->getPathPose(tpsPt.k, relTrgStep);
            const auto absPose         = from.state.pose + relReconstrPose;

            // out of lattice limits?
            if (absPose.x < grid_.getXMin() || absPose.y < grid_.getYMin() ||
                absPose.phi < grid_.getPhiMin())
                continue;
            if (absPose.x > grid_.getXMax() || absPose.y > grid_.getYMax() ||
                absPose.phi > grid_.getPhiMax())
                continue;

            const NodeCoords nc = nodeGridCoords(absPose);

            auto& path = bestPaths[nodeCoordsToAbsIndex(nc)];

            // Ok, it's a valid new neighbor with this PTG.
            // Is it shorter with this PTG than with others?
            if (dist < path.ptgDist)
            {
                path.ptgDist            = dist;
                path.ptgIndex           = ptgIdx;
                path.ptgTrajIndex       = tpsPt.k;
                path.relReconstrPose    = relReconstrPose;
                path.neighborNodeCoords = nc;
            }
        }

    }  // end for each PTG

    // Fill "neighbors" from valid "bestPaths":
    list_paths_to_neighbors_t neighbors;

    for (const auto& kv : bestPaths)
    {
        const auto& path = kv.second;

        if (!path.ptgIndex.has_value()) continue;  // skip

        neighbors.emplace_back(kv.second);
    }

    return neighbors;
}

mrpt::maps::CPointsMap::Ptr TPS_Astar::cached_local_obstacles(
    const mrpt::math::TPose2D&                      queryPose,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
    double                                          MAX_PTG_XY_DIST)
{
    MRPT_TODO("Impl actual cache");

    auto obs = mrpt::maps::CSimplePointsMap::Create();

    for (const auto& obs : globalObstacles)
    {
        ASSERT_(obs);
        transform_pc_square_clipping(
            *obs, mrpt::poses::CPose2D(queryPose), MAX_PTG_XY_DIST, *obs);
    }

    return obs;
}
