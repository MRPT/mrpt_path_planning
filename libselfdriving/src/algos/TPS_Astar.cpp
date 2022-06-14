/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/algos/render_tree.h>
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

    return c;
}

void TPS_Astar_Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_OPT(c, SE2_metricAngleWeight);
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
        auto& n = getOrCreateNodeByPose(in.stateStart.pose, tree);
        n.state = in.stateStart;

        std::cout << "Created start node ID=" << n.id.value() << std::endl;

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
    auto& nodeGoal = getOrCreateNodeByPose(in.stateGoal.pose, tree);
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

    std::cout << "Created goal node ID=" << nodeGoal.id.value() << std::endl;

    while (!openSet.empty())
    {
        // node with the lowest fScore:
        Node& current = *openSet.begin()->second.ptr;

        // current==goal?
        if (current.id.value() == nodeGoal.id.value())
        {
            std::cout << "Reached goal cell pose:" << current.state.pose
                      << std::endl;

            // Path found:
            break;
        }

        // remove it from open set:
        current.pendingInOpenSet = false;
        openSet.erase(openSet.begin());

        // for each neighbor of current:
        const auto neighbors =
            find_feasible_paths_to_neighbors(current, in.ptgs);

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
                    edge.ptgTrajIndex.value(), edge.distance, ptg_step);
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

            MoveEdgeSE2_TPS tentativeEdge;

            tentativeEdge.parentId     = current.id.value();
            tentativeEdge.ptgDist      = edge.distance;
            tentativeEdge.ptgIndex     = edge.ptgIndex.value();
            tentativeEdge.ptgPathIndex = edge.ptgTrajIndex.value();
            MRPT_TODO("targetRelSpeed?");
            // tentativeEdge.targetRelSpeed = ds.targetRelSpeed;
            tentativeEdge.stateFrom = current.state;
            tentativeEdge.stateTo   = x_i;
            // interpolated path:
            if (const auto nSeg = params_.pathInterpolatedSegments; nSeg > 0)
            {
                auto& ip = tentativeEdge.interpolatedPath.emplace();
                ip.emplace_back(0, 0, 0);  // fixed
                // interpolated:
                for (size_t i = 0; i < nSeg; i++)
                {
                    const auto iStep =
                        ((i + 1) * tentativeEdge.ptgPathIndex) / (nSeg + 2);
                    ip.emplace_back(
                        ptg.getPathPose(tentativeEdge.ptgPathIndex, iStep));
                }
                ip.emplace_back(reconstrRelPose);  // already known
            }

            // Let's compute its cost:
            tentativeEdge.cost = cost_path_segment(tentativeEdge);
            ASSERT_GT_(tentativeEdge.cost, .0);

            const cost_t tentative_gScore = current.gScore + tentativeEdge.cost;

            auto& neighborNode = getOrCreateNodeByPose(x_i.pose, tree);

            if (tentative_gScore < neighborNode.gScore)
            {
                std::cout << "New better path to:" << x_i.pose
                          << " cost:" << tentative_gScore << std::endl;

                const bool hasToRewire = neighborNode.cameFrom.has_value();

                // This path to neighbor is better than any previous one,
                // keep it:
                neighborNode.cameFrom = &current;
                neighborNode.gScore   = tentative_gScore;

                // fScore[neighbor] := tentative_gScore + h(neighbor)
                neighborNode.fScore =
                    tentative_gScore +
                    heuristic(current.state, neighborNode.state);

                if (!neighborNode.pendingInOpenSet)
                {
                    neighborNode.pendingInOpenSet = true;
                    openSet.insert({neighborNode.fScore, &neighborNode});

                    std::cout << "+openSet x=" << x_i.pose << std::endl;
                }

                // Delete old edge, if any:
                if (hasToRewire)
                {
                    tree.rewire_node_parent(
                        neighborNode.id.value(), tentativeEdge);
                }
                else
                {
                    // Add edge to tree:
                    tree.insert_node_and_edge(
                        tentativeEdge.parentId, neighborNode.id.value(),
                        neighborNode.state, tentativeEdge);
                }
            }
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

distance_t TPS_Astar::heuristic(
    const SE2_KinState& from, const SE2_KinState& goal) const
{
    selfdriving::PoseDistanceMetric_Lie<selfdriving::SE2_KinState> metric(
        params_.SE2_metricAngleWeight);

    return metric.distance(from.pose, goal.pose);
}

TPS_Astar::list_paths_to_neighbors_t
    TPS_Astar::find_feasible_paths_to_neighbors(
        const TPS_Astar::Node& from, const TrajectoriesAndRobotShape& trs)
{
    const auto iFromCoords = nodeGridCoords(from.state.pose);

    // Init ptgs:
    for (auto& ptg : trs.ptgs)
    {
        ptg_t::TNavDynamicState ds;
        (ds.curVelLocal = from.state.vel).rotate(-from.state.pose.phi);

        MRPT_TODO("Include target node speed?");
        ds.relTarget      = {1.0, 0, 0};
        ds.targetRelSpeed = 1.0;

        ptg->updateNavDynamicState(ds);
    }

    // Create all neighbors:
    std::map<absolute_cell_index_t, path_to_neighbor_t> bestPaths;
    for (int ix = -1; ix <= 1; ix++)
    {
        for (int iy = -1; iy <= 1; iy++)
        {
            for (int ip = -1; ip <= 1; ip++)
            {
                // skip the (0,0,0) incremental motion:
                if (ix == 0 && iy == 0 && ip == 0) continue;

                const NodeCoords nc = iFromCoords + NodeCoords(ix, iy, ip);
                bestPaths[nodeCoordsToAbsIndex(nc)].nodeCoords = nc;
            }
        }
    }

    // Evaluate which PTG takes us to each neigbor cell:
    for (auto& kv : bestPaths)
    {
        // take references for writing best found ptg paths:
        auto& path = kv.second;

        for (size_t curPtgIdx = 0; curPtgIdx < trs.ptgs.size(); curPtgIdx++)
        {
            auto& ptg = trs.ptgs.at(curPtgIdx);

            // ptg->inverseMap_WS2TP()
            const auto neighNodePose = nodeCoordsToPose(path.nodeCoords);

            const auto            relPose = neighNodePose - from.state.pose;
            int                   relTrg_k;
            normalized_distance_t relTrg_d;
            if (ptg->inverseMap_WS2TP(relPose.x, relPose.y, relTrg_k, relTrg_d))
            {
                // valid. Is it better than existing?
                distance_t dist = relTrg_d * ptg->getRefDistance();

                if (dist < path.distance)
                {
                    path.ptgIndex     = curPtgIdx;
                    path.ptgTrajIndex = relTrg_k;
                    path.distance     = dist;
                }
            }
        }
    }

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
