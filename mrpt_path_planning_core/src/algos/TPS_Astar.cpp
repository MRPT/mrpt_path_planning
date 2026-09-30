/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/algos/reeds_shepp.h>
#include <mpp/algos/render_tree.h>
#include <mpp/algos/tp_obstacles_single_path.h>
#include <mpp/algos/transform_pc_square_clipping.h>
#include <mpp/algos/within_bbox.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/ptgs/DiffDriveCollisionGridBased.h>
#include <mpp/ptgs/DiffDrive_C.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/version.h>
#include <mrpt/viz/Scene.h>

#include <cmath>
#include <iostream>
#include <queue>
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
    MCP_SAVE(c, heuristic_epsilon);
    MCP_SAVE(c, use_analytic_expansion);
    MCP_SAVE(c, use_reeds_shepp_heuristic);
    MCP_SAVE(c, use_reeds_shepp_expansion);
    MCP_SAVE(c, reeds_shepp_expansion_max_length);
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
    MCP_LOAD_OPT(c, heuristic_epsilon);
    MCP_LOAD_OPT(c, use_analytic_expansion);
    MCP_LOAD_OPT(c, use_reeds_shepp_heuristic);
    MCP_LOAD_OPT(c, use_reeds_shepp_expansion);
    MCP_LOAD_OPT(c, reeds_shepp_expansion_max_length);

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
        mrpt::keep_max(MAX_XY_DIST, obstacle_clipping_distance(*ptg));
    ASSERT_(MAX_XY_DIST > 0);

    // Cache max linear speed for heuristic unit conversion (distance→time).
    maxLinSpeed_ = 0.0;
    for (const auto& ptg : in.ptgs.ptgs)
        mrpt::keep_max(maxLinSpeed_, ptg->getMaxLinVel());
    if (maxLinSpeed_ <= 0) maxLinSpeed_ = 1.0;  // fallback: units = distance

    reedsSheppTurningRadius_ = 0.0;
    if (params_.use_reeds_shepp_heuristic)
    {
        reedsSheppTurningRadius_ = reeds_shepp_turning_radius(in.ptgs);
        if (reedsSheppTurningRadius_ <= 0)
        {
            MRPT_LOG_WARN(
                "use_reeds_shepp_heuristic ignored: it requires all PTGs to "
                "be of type DiffDrive_C.");
        }
    }

    rsExpansionRadius_ = 0.0;
    if (params_.use_reeds_shepp_expansion && params_.use_analytic_expansion &&
        !prepare_reeds_shepp_expansion(in.ptgs))
    {
        MRPT_LOG_DEBUG(
            "Reeds-Shepp shot not used: it requires forward and reverse "
            "DiffDrive_C PTGs with an odd number of trajectories.");
    }

    // obstacles (TODO: dynamic over future time?):
    std::vector<mrpt::maps::CPointsMap::Ptr> obstaclePoints;
    for (const auto& os : in.obstacles)
    {
        if (!os) continue;  // should never happen?

        // apply clipping for efficiency:
        // (z is arbitrary and ignored inside)
        os->apply_clipping_box(mrpt::math::TBoundingBox(
            {in.worldBboxMin.x, in.worldBboxMin.y, -1.0},
            {in.worldBboxMax.x, in.worldBboxMax.y, 1.0}));

        // Get obstacles:
        obstaclePoints.emplace_back(os->obstacles());
    }

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    grid_.clear();
    localObstaclesCache_.clear();

    // ----------------------------------------
    //
    // A* algorithm
    //
    // ----------------------------------------
    // Open set, ordered by fScore; ties are popped in insertion order (FIFO),
    // as a std::multimap keyed by fScore would do, but without one heap
    // allocation per insertion.
    struct OpenEntry
    {
        distance_t fScore;
        uint64_t   seq;
        Node*      node;
        bool       operator>(const OpenEntry& o) const
        {
            return fScore != o.fScore ? fScore > o.fScore : seq > o.seq;
        }
    };
    std::priority_queue<
        OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>>
                          openSet;
    uint64_t              openSeq    = 0;
    mrpt::graphs::TNodeID nextFreeId = 0;

    // openSet <- startNode
    {
        auto& n = getOrCreateNodeByPose(in.stateStart, nextFreeId);
        n.state = in.stateStart;

        //   X_T ← {X_0 }    # Tree nodes (state space)
        // ------------------------------------------------------------------
        tree.root = n.id.value();
        tree.insert_root_node(tree.root, n.state);

        n.gScore = 0;
        n.fScore = params_.heuristic_epsilon * heuristic(n.state, in.stateGoal);
        n.pendingInOpenSet = true;

        openSet.push({n.fScore, openSeq++, &n});
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

    // Desired speed at goal, stored as **absolute linear speed in m/s**.
    // Each PTG normalises this against its own v_max when it reads the map
    // (see find_feasible_paths_to_neighbors).  Only the XY linear component
    // is used; a purely-angular exit velocity maps to 0 (stop) because
    // PTG targetRelSpeed is a linear-speed modifier.
    nodes_with_desired_speed_t nodesWithDesiredSpeed;
    nodesWithDesiredSpeed[goalCellIndices] =
        std::hypot(in.stateGoal.vel.vx, in.stateGoal.vel.vy);

    unsigned int nIter = 0;

    double tLastCallback = planInitTime;

    // Analytic goal expansion (deferred, optimality-preserving): when a
    // collision-free edge lands in the goal cell we remember it as a
    // *candidate* solution instead of terminating immediately. Stopping on the
    // first such edge is greedy and can produce grossly suboptimal paths: with
    // an SE(2) goal, a node that arrived at the goal xy but a couple of yaw
    // cells off can only re-enter the exact goal cell through a
    // near-full-circle PTG arc (with a tight turning radius), so that first
    // edge is a ~360 deg loop (see fig:cases in the paper). We instead keep the
    // cheapest goal-landing candidate and only commit once its actual cost is
    // provably (eps-)optimal, i.e. <= the minimum fScore still pending in the
    // open set (committed at the top of the loop). This preserves the
    // early-termination speed-up without accepting the loop, and is a no-op for
    // R(2) point goals (verified identical on the 300-world BARN sweep).
    Node*  bestGoalCandidate  = nullptr;
    cost_t bestGoalCandidateG = std::numeric_limits<cost_t>::max();
    // A goal candidate may also be the last node of a Reeds-Shepp shot, which
    // lives in the tree only (not in the lattice):
    std::optional<mrpt::graphs::TNodeID> bestGoalCandidateShotId;

    // Defer building per-edge interpolated paths during the search: it is a
    // major cost (a std::map per edge) and is only needed for (a) cost
    // evaluators that score edges, (b) debug visualization, or (c) progress
    // callbacks that inspect the partial path. When none of these apply, we
    // store only estimatedExecTime during the search and interpolate just the
    // final solution edges at the end.
    const bool deferInterpolation =
        costEvaluators_.empty() &&
        params_.saveDebugVisualizationDecimation == 0 && !progressCallback_;

    // Tries a Reeds-Shepp shot from `n` to the goal (see
    // use_reeds_shepp_expansion). A clear shot becomes the goal candidate if
    // it is cheaper than the current one or, with `replace`, in any case (an
    // exact goal is preferred to a candidate anywhere in the goal cell).
    // Returns whether the shot is clear.
    auto tryReedsSheppShot = [&](const Node& n, bool replace)
    {
        const auto shot = reeds_shepp_shot(
            n, in.stateGoal, in.ptgs, obstaclePoints, MAX_XY_DIST, in);
        if (shot.empty()) { return false; }
        cost_t g = n.gScore;
        for (const auto& e : shot) { g += e.edge.cost; }
        if (!replace && g >= bestGoalCandidateG) { return true; }

        mrpt::graphs::TNodeID parent = n.id.value();
        for (const auto& e : shot)
        {
            const auto id       = nextFreeId++;
            auto       edgeData = e.edge;
            edgeData.parentId   = parent;
            tree.insert_node_and_edge(parent, id, e.stateTo, edgeData);
            parent = id;
        }
        bestGoalCandidate       = nullptr;
        bestGoalCandidateG      = g;
        bestGoalCandidateShotId = parent;
        return true;
    };

    while (!openSet.empty())
    {
        mrpt::system::CTimeLoggerEntry tle(profiler_(), "plan.iter");

        nIter++;  // just for debugging purposes

        // node with the lowest fScore:
        Node& current = *openSet.top().node;

        // Skip stale entries: this node was already expanded via a
        // earlier (better) open-set entry.
        if (current.visited)
        {
            openSet.pop();
            continue;
        }

        // Deferred analytic expansion: if we hold a goal candidate whose actual
        // cost is no worse than the best possible cost still in the open set
        // (current.fScore is the minimum pending fScore), no remaining path can
        // beat it, so commit to it now. See the note above.
        if (params_.use_analytic_expansion && bestGoalCandidateShotId &&
            bestGoalCandidateG <= current.fScore)
        {
            po.goalNodeId           = *bestGoalCandidateShotId;
            po.bestNodeId           = po.goalNodeId;
            po.bestNodeIdCostToGoal = 0;
            MRPT_LOG_DEBUG("Reeds-Shepp shot committed as the solution.");
            break;
        }
        if (params_.use_analytic_expansion && bestGoalCandidate != nullptr &&
            bestGoalCandidateG <= current.fScore)
        {
            Node& gn = *bestGoalCandidate;
            // Reach the goal exactly from the candidate, if possible. The
            // shot is then committed by the rule above.
            if (rsExpansionRadius_ > 0 && tryReedsSheppShot(gn, true))
            {
                continue;
            }
            if (in.stateGoal.state.isPoint())
            {
                const auto& goalPt = in.stateGoal.state.point();
                gn.state.pose.x    = goalPt.x;
                gn.state.pose.y    = goalPt.y;
            }
            else { gn.state.pose = in.stateGoal.state.pose(); }
            tree.node_state(*gn.id).pose = gn.state.pose;

            nodeGoal                = &gn;
            po.goalNodeId           = gn.id.value();
            po.bestNodeId           = po.goalNodeId;
            po.bestNodeIdCostToGoal = 0;

            MRPT_LOG_DEBUG_STREAM(
                "Analytic expansion: optimal goal candidate committed at "
                << gn.state.asString());
            break;
        }

        // current==goal?
        // we must check the state to be on the same lattice cell to check
        // for a match of the current SE(2) pose against the goal state,
        // which may be either a SE(2) pose or a R2 point:
        //
        // With the Reeds-Shepp shot, a node in the goal cell is first used to
        // reach the goal exactly: if its shot is clear, or a shot candidate
        // already exists, it is expanded as any other node and a shot is
        // committed by the rule above. Otherwise the search ends here, as
        // without shots, before any goal-cell node has been expanded.
        if (const auto curNodeGridIdx = nodeGridCoords(current.state.pose);
            curNodeGridIdx.sameLocation(goalCellIndices) &&
            !(rsExpansionRadius_ > 0 && (bestGoalCandidateShotId.has_value() ||
                                         tryReedsSheppShot(current, true))))
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
        openSet.pop();
        po.numExpandedNodes++;

        // for each neighbor of current:
        const auto neighbors = find_feasible_paths_to_neighbors(
            current, in.ptgs, in.stateGoal, obstaclePoints, MAX_XY_DIST,
            nodesWithDesiredSpeed, in.worldBboxMin, in.worldBboxMax);

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
            // Note: phi is NOT checked here since it lives on S^1 (wraps
            // at ±π) and cannot meaningfully be bounded linearly.
            if (q_i.x < in.worldBboxMin.x || q_i.y < in.worldBboxMin.y)
                continue;
            if (q_i.x > in.worldBboxMax.x || q_i.y > in.worldBboxMax.y)
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

            newEdge.ptgTrimmableSpeed = edge.ptgTrimmableSpeed;
            newEdge.ptgFinalGoalRelSpeed =
                edge.ptgDynState.value().targetRelSpeed;
            newEdge.ptgFinalRelativeGoal =
                in.stateGoal.asSE2KinState().pose - current.state.pose;

#if MRPT_VERSION >= 0x20e02  // >=2.14.2
            newEdge.ptgInternalState =
                ptg.getCurrentNavDynamicState().internalState;
#endif
            newEdge.stateFrom    = current.state;
            newEdge.stateTo      = x_i;
            newEdge.ptgStepIndex = ptg_step;

            // interpolated path (deferred to the final solution when possible,
            // see `deferInterpolation`): otherwise set the cheap exec-time
            // only.
            if (deferInterpolation)
            {
                newEdge.estimatedExecTime =
                    ptg_step * ptg.getPathStepDuration();
            }
            else
            {
                edge_interpolated_path(
                    newEdge, in.ptgs, reconstrRelPose, ptg_step,
                    params_.pathInterpolatedSegments);
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

            // fScore[neighbor] := tentative_gScore + eps * h(neighbor).
            // Weighted A* (eps>=1): inflating only the OPEN ordering, NOT the
            // raw costToGoal kept below for best-node tracking, yields a
            // solution within eps of optimal (ARA*/SBPL-style).
            const cost_t costToGoal =
                heuristic(neighborNode.state, in.stateGoal);
            neighborNode.fScore =
                tentative_gScore + params_.heuristic_epsilon * costToGoal;

            // Always (re-)insert into the open set with the updated
            // fScore. If an older entry with a higher fScore remains,
            // it will be skipped when popped via the visited check.
            neighborNode.pendingInOpenSet = true;
            openSet.push({neighborNode.fScore, openSeq++, &neighborNode});

            // Overwrite state with new one:
            neighborNode.state = x_i;

            // Delete old edge, if any:
            if (hasToRewire)
            {
                // do rewire:
                tree.rewire_node_parent(neighborNode.id.value(), newEdge);

                // The cell's representative state changed too: keep the tree
                // node consistent with its new incoming edge, since returned
                // paths are read from the tree.
                auto& treeNode = tree.node_state(neighborNode.id.value());
                treeNode.pose  = neighborNode.state.pose;
                treeNode.vel   = neighborNode.state.vel;
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

            // Analytic expansion: this accepted edge connects (collision-free)
            // into the goal cell. Remember it as a candidate solution (keeping
            // the cheapest); we only commit to it once it is provably optimal,
            // at the top of the while loop. See the note where
            // bestGoalCandidate is declared.
            if (params_.use_analytic_expansion &&
                edge.neighborNodeCoords.sameLocation(goalCellIndices) &&
                neighborNode.gScore < bestGoalCandidateG)
            {
                bestGoalCandidate  = &neighborNode;
                bestGoalCandidateG = neighborNode.gScore;
                bestGoalCandidateShotId.reset();
            }

        }  // end for each edge to neighbor

        // Reeds-Shepp shot to the goal (see use_reeds_shepp_expansion), if
        // a lower bound of its cost can improve on the current candidate:
        if (rsExpansionRadius_ > 0)
        {
            const double lowerBoundLength =
                in.stateGoal.state.isPose()
                    ? reeds_shepp_distance(
                          current.state.pose, in.stateGoal.state.pose(),
                          rsExpansionRadius_)
                    : (current.state.pose.translation() -
                       in.stateGoal.state.point())
                          .norm();
            if (current.gScore + lowerBoundLength / maxLinSpeed_ <
                    bestGoalCandidateG &&
                lowerBoundLength <= params_.reeds_shepp_expansion_max_length)
            {
                tryReedsSheppShot(current, false);
            }
        }

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
            mrpt::viz::Scene scene;
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

    // If interpolation was deferred during the search, build it now for just
    // the final solution-path edges (needed for output / refinement / exec).
    if (deferInterpolation && po.bestNodeId.has_value() &&
        po.goalNodeId == po.bestNodeId)
    {
        const auto [solNodes, solEdges] =
            tree.backtrack_path(po.bestNodeId.value());
        for (auto* e : solEdges)
        {
            if (e == nullptr) { continue; }
            // Recover the relative reconstruction pose (stateTo in the frame
            // of stateFrom); identical to what was used during the search.
            const auto reconstrRelPose = e->stateTo.pose - e->stateFrom.pose;
            edge_interpolated_path(
                *e, in.ptgs, reconstrRelPose, e->ptgStepIndex,
                params_.pathInterpolatedSegments);
        }
    }

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

double TPS_Astar::reeds_shepp_turning_radius(
    const TrajectoriesAndRobotShape& trs)
{
    double r = std::numeric_limits<double>::max();
    for (const auto& ptg : trs.ptgs)
    {
        const auto* c = dynamic_cast<const ptg::DiffDrive_C*>(ptg.get());
        if (!c || c->getMax_W() <= 0) { return 0; }
        mrpt::keep_min(r, c->getMax_V() / c->getMax_W());
    }
    return trs.ptgs.empty() ? 0 : r;
}

cost_t TPS_Astar::default_heuristic_SE2(
    const SE2_KinState& from, const mrpt::math::TPose2D& goal) const
{
    if (reedsSheppTurningRadius_ > 0)
    {
        return reeds_shepp_distance(from.pose, goal, reedsSheppTurningRadius_) /
               maxLinSpeed_;
    }

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

    cost_t h = (distSE2 + params_.heuristic_heading_weight * distHeading) /
               maxLinSpeed_;

    return h;
}

cost_t TPS_Astar::default_heuristic_R2(
    const SE2_KinState& from, const mrpt::math::TPoint2D& goal) const
{
    // Distance in R^2 only — heading is irrelevant for R(2) goals.
    cost_t h = (from.pose.translation() - goal).norm() / maxLinSpeed_;

    return h;
}

double TPS_Astar::obstacle_clipping_distance(const ptg_t& ptg)
{
    if (const auto* g =
            dynamic_cast<const ptg::DiffDriveCollisionGridBased*>(&ptg);
        g)
    {
        return g->getObstacleReachDistance();
    }
    return ptg.getRefDistance() + ptg.getMaxRobotRadius();
}

TPS_Astar::Node& TPS_Astar::getOrCreateNodeByPose(
    const SE2_KinState& p, mrpt::graphs::TNodeID& nextFreeId)
{
    auto& n = grid_[NodeCoords(nodeGridCoords(p.pose))];

    if (!n.id.has_value())
    {
        n.id    = nextFreeId++;
        n.state = p;
    }
    return n;
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
        const nodes_with_desired_speed_t& nodesWithSpeed,
        const mrpt::math::TPose2D&        worldBboxMin,
        const mrpt::math::TPose2D&        worldBboxMax)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "find_feasible");

    // const auto iFromCoords = nodeGridCoords(from.state.pose);

    const NodeCoords iGoalCoords = goalState.state.isPoint()
                                       ? nodeGridCoords(goalState.state.point())
                                       : nodeGridCoords(goalState.state.pose());

    const auto relGoal = goalState.asSE2KinState().pose - from.state.pose;

    const double halfCell = params_.grid_resolution_xy * 0.5;

    // local obstacles as seen from this "from" pose:
    const size_t nLocalObs = local_obstacles_to_buffers(
        from.state.pose, globalObstacles, MAX_XY_OBSTACLES_CLIPPING_DIST);

    // If two PTGs reach the same cell, keep the shortest/best:
    std::unordered_map<NodeCoords, path_to_neighbor_t, NodeCoordsHash>
        bestPaths;

#if 0
    size_t totalConsidered = 0, totalCollided = 0;
#endif
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
                // it->second is an absolute speed (m/s); normalise against
                // this PTG's own maximum linear velocity so that the result
                // is in [0, 1] regardless of the robot's top speed.
                const double ptgVmax = std::max(ptg->getMaxLinVel(), 1e-6);
                ds.targetRelSpeed    = std::min(1.0, it->second / ptgVmax);
            }
            else { ds.targetRelSpeed = 0; }

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
            const double queryTolerance = params_.grid_resolution_xy;
            const auto   trgInvMap =
                ptg->inverseMap_WS2TP(relGoal.x, relGoal.y, queryTolerance);
            if (trgInvMap.has_value())
            {
                const int                   relTrg_k    = trgInvMap->first;
                const normalized_distance_t relTrg_d    = trgInvMap->second;
                ptg_step_t                  relTrg_step = 0;
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

        // Build the full TP-obstacle free-distance array ONCE for this PTG at
        // this node, in a single pass over the local obstacle cloud:
        // updateTPObstacle() fills the collision-free distance for ALL
        // trajectory directions `k` per obstacle point (one collision-grid
        // lookup yields all k). This replaces the former pattern of re-scanning
        // the whole cloud once per candidate trajectory, which was the dominant
        // planner cost (O(candidates x N_points) -> O(N_points)). The collision
        // grid is purely geometric, so the result is independent of the
        // trimmable speed and of the timestamp, and can be shared by all
        // candidates of this PTG.
        std::vector<double> tpObstacles;
        {
            mrpt::system::CTimeLoggerEntry tleObsAll(
                profiler_(), "find_feasible.tp_obstacles_all");

            ptg->initTPObstacles(tpObstacles);
            if (const auto* gridPtg =
                    dynamic_cast<const ptg::DiffDriveCollisionGridBased*>(
                        ptg.get()))
            {
                gridPtg->updateTPObstacles(
                    localObsX_.data(), localObsY_.data(), nLocalObs,
                    tpObstacles);
            }
            else
            {
                for (size_t i = 0; i < nLocalObs; i++)
                {
                    ptg->updateTPObstacle(
                        localObsX_[i], localObsY_[i], tpObstacles);
                }
            }
        }

        // now, check which ones of those paths are not blocked by
        // obstacles:
        for (size_t tpsPtIdx = 0; tpsPtIdx < tpsPointsToConsider.size();
             tpsPtIdx++)
        {
            const auto& tpsPt = tpsPointsToConsider[tpsPtIdx];

#if 0
            totalConsidered++;
#endif

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
            // Note: phi is NOT checked since it lives on S^1 and cannot
            // be meaningfully bounded with a linear comparison.
            if (absPose.x < worldBboxMin.x || absPose.y < worldBboxMin.y)
                continue;
            if (absPose.x > worldBboxMax.x - halfCell ||
                absPose.y > worldBboxMax.y - halfCell)
                continue;

            const NodeCoords nc = nodeGridCoords(absPose);

            // check for collisions: read the precomputed free distance for
            // this trajectory direction (built once above for all candidates).
            const distance_t freeDistance = tpObstacles[tpsPt.k];

            if (relTrgDist >= freeDistance)
            {
                // we would need to move farther away than what is possible
                // without colliding: discard this trajectory.
#if 0
                totalCollided++;
#endif
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

            auto& path = bestPaths[nc];

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
                // Must store the speed that was active during collision
                // evaluation (applied via ptgTrimmable->trimmableSpeed_ above);
                // without this, ptgTrimmableSpeed keeps its default of 1.0 and
                // the trimmed speed used to win this best-path slot is lost.
                path.ptgTrimmableSpeed = tpsPt.speed;
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

    const auto clippedGlobal =
        clipped_global_obstacles(queryPose, globalObstacles, MAX_PTG_XY_DIST);

    // Per-call: rigid-transform the small clipped subset into the robot-local
    // frame of `queryPose` (translation + rotation by heading).
    auto         local = mrpt::maps::CSimplePointsMap::Create();
    const auto   inv   = -mrpt::poses::CPose2D(queryPose);
    const auto&  xs    = clippedGlobal->getPointsBufferRef_x();
    const auto&  ys    = clippedGlobal->getPointsBufferRef_y();
    const size_t n     = clippedGlobal->size();
    local->reserve(n);
    for (size_t i = 0; i < n; i++)
    {
        double ox = 0, oy = 0;
        inv.composePoint(xs[i], ys[i], ox, oy);
        local->insertPointFast(ox, oy, 0);
    }
    return local;
}

size_t TPS_Astar::local_obstacles_to_buffers(
    const mrpt::math::TPose2D&                      queryPose,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
    double                                          MAX_PTG_XY_DIST)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "cached_local_obstacles");

    const auto clippedGlobal =
        clipped_global_obstacles(queryPose, globalObstacles, MAX_PTG_XY_DIST);

    // Same transform (and float storage) as cached_local_obstacles():
    const auto   inv = -mrpt::poses::CPose2D(queryPose);
    const auto&  xs  = clippedGlobal->getPointsBufferRef_x();
    const auto&  ys  = clippedGlobal->getPointsBufferRef_y();
    const size_t n   = clippedGlobal->size();
    localObsX_.resize(n);
    localObsY_.resize(n);
    for (size_t i = 0; i < n; i++)
    {
        double ox = 0, oy = 0;
        inv.composePoint(xs[i], ys[i], ox, oy);
        localObsX_[i] = static_cast<float>(ox);
        localObsY_[i] = static_cast<float>(oy);
    }
    return n;
}

mrpt::maps::CPointsMap::Ptr TPS_Astar::clipped_global_obstacles(
    const mrpt::math::TPose2D&                      queryPose,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
    double                                          MAX_PTG_XY_DIST)
{
    // Only the *clipping* of the global obstacle set to a local window depends
    // solely on the xy cell, so that (expensive, O(N_global)) step is cached
    // per (ix,iy). The subsequent rigid transform into the robot-local frame
    // DEPENDS ON HEADING and must NOT be cached across headings: doing so
    // (former behavior, keyed by xy only) reused obstacles rotated for a
    // different heading, yielding collision FALSE NEGATIVES (the planner could
    // tunnel through walls). So we transform the small clipped subset per call.
    const NodeCoords key(x2idx(queryPose.x), y2idx(queryPose.y));

    mrpt::maps::CPointsMap::Ptr clippedGlobal;
    if (auto it = localObstaclesCache_.find(key);
        it != localObstaclesCache_.end())
    {
        clippedGlobal = it->second;
    }
    else
    {
        // Clip (keeping GLOBAL coordinates) to a square around the cell center,
        // enlarged by half a cell so the cached subset is a valid superset for
        // any exact pose falling in this cell.
        const double res = params_.grid_resolution_xy;
        const double cx  = key.idxX * res;
        const double cy  = key.idxY * res;
        const double win = MAX_PTG_XY_DIST + 0.5 * res;

        clippedGlobal = mrpt::maps::CSimplePointsMap::Create();
        for (const auto& obs : globalObstacles)
        {
            ASSERT_(obs);
            const auto&  xs = obs->getPointsBufferRef_x();
            const auto&  ys = obs->getPointsBufferRef_y();
            const size_t n  = obs->size();
            for (size_t i = 0; i < n; i++)
            {
                if (std::abs(xs[i] - cx) <= win && std::abs(ys[i] - cy) <= win)
                {
                    clippedGlobal->insertPointFast(xs[i], ys[i], 0);
                }
            }
        }
        localObstaclesCache_.emplace(key, clippedGlobal);
    }
    return clippedGlobal;
}

bool TPS_Astar::prepare_reeds_shepp_expansion(
    const TrajectoriesAndRobotShape& trs)
{
    rsForward_         = RsExpansionPtg();
    rsReverse_         = RsExpansionPtg();
    rsExpansionRadius_ = 0.0;
    double radius      = 0.0;
    for (size_t i = 0; i < trs.ptgs.size(); i++)
    {
        const auto& ptg = *trs.ptgs.at(i);
        if (!dynamic_cast<const ptg::DiffDrive_C*>(&ptg)) { return false; }
        const auto n = ptg.getPathCount();
        if (n < 3 || n % 2 == 0) { return false; }
        RsExpansionPtg m;
        m.ptgIndex  = i;
        m.kStraight = static_cast<trajectory_index_t>((n - 1) / 2);
        // The two extreme trajectories are the tightest arcs; which one turns
        // left (positive curvature) depends on the driving direction.
        const auto   twStraight = ptg.getPathTwist(m.kStraight, 0);
        const auto   tw0        = ptg.getPathTwist(0, 0);
        const double curv0      = tw0.omega / tw0.vx;
        if (std::abs(twStraight.omega) > 1e-9 || twStraight.vx == 0)
        {
            return false;
        }
        m.kLeft        = curv0 > 0 ? 0 : static_cast<trajectory_index_t>(n - 1);
        m.kRight       = curv0 > 0 ? static_cast<trajectory_index_t>(n - 1) : 0;
        m.valid        = true;
        const double r = 1.0 / std::abs(curv0);
        if (radius > 0 && std::abs(r - radius) > 1e-6 * radius)
        {
            return false;  // PTGs with different tightest radii
        }
        radius                                        = r;
        (twStraight.vx > 0 ? rsForward_ : rsReverse_) = m;
    }
    if (!rsForward_.valid || !rsReverse_.valid) { return false; }
    rsExpansionRadius_ = radius;
    return true;
}

std::vector<TPS_Astar::RsShotEdge> TPS_Astar::reeds_shepp_shot(
    const Node& from, const SE2orR2_KinState& goal,
    const TrajectoriesAndRobotShape&                trs,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& globalObstacles,
    double MAX_XY_OBSTACLES_CLIPPING_DIST, const PlannerInput& in)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_(), "reeds_shepp_shot");

    std::vector<RsShotEdge> out;
    const auto              segments =
        goal.state.isPose()
                         ? reeds_shepp_path(
                               from.state.pose, goal.state.pose(), rsExpansionRadius_)
                         : reeds_shepp_path_to_point(
                               from.state.pose, goal.state.point(), rsExpansionRadius_);
    double length = 0;
    for (const auto& seg : segments) { length += std::abs(seg.length); }
    if (segments.empty() || length > params_.reeds_shepp_expansion_max_length)
    {
        return out;
    }

    SE2_KinState        state = from.state;
    std::vector<double> tpObstacles;
    for (const auto& seg : segments)
    {
        const auto& m   = seg.length > 0 ? rsForward_ : rsReverse_;
        const auto& ptg = *trs.ptgs.at(m.ptgIndex);
        const auto  k   = seg.type == 'L'   ? m.kLeft
                          : seg.type == 'R' ? m.kRight
                                            : m.kStraight;
        // C-PTG trajectories have constant speed, so the step reached after
        // an arc length s is s / (v dt). (The PTG "distance" also accounts for
        // rotation, so it is not the arc length.) Segments longer than the
        // stored trajectory are split into chunks.
        const double stepLength =
            std::abs(ptg.getPathTwist(k, 0).vx) * ptg.getPathStepDuration();
        const double totalSteps = std::abs(seg.length) / stepLength;
        const double maxSteps   = 0.95 * (ptg.getPathStepCount(k) - 1);
        const auto   nChunks =
            static_cast<size_t>(std::ceil(totalSteps / maxSteps));
        for (size_t c = 0; c < nChunks; c++)
        {
            // Rounded at each chunk end, so rounding errors do not add up:
            const auto s0 = static_cast<uint32_t>(
                std::lround(totalSteps * double(c) / double(nChunks)));
            const auto s1 = static_cast<uint32_t>(
                std::lround(totalSteps * double(c + 1) / double(nChunks)));
            const auto step = std::optional<uint32_t>(s1 - s0);
            if (*step == 0) { continue; }
            const double dist = ptg.getPathDist(k, *step);

            // Certified collision check from the current pose:
            const size_t nObs = local_obstacles_to_buffers(
                state.pose, globalObstacles, MAX_XY_OBSTACLES_CLIPPING_DIST);
            const auto* gridPtg =
                dynamic_cast<const ptg::DiffDriveCollisionGridBased*>(&ptg);
            ASSERT_(gridPtg);
            gridPtg->initTPObstacles(tpObstacles);
            gridPtg->updateTPObstacles(
                localObsX_.data(), localObsY_.data(), nObs, tpObstacles);
            if (dist >= tpObstacles[k])
            {
                out.clear();
                return out;
            }

            SE2_KinState next;
            next.pose = state.pose + ptg.getPathPose(k, *step);
            (next.vel = ptg.getPathTwist(k, *step)).rotate(state.pose.phi);
            if (next.pose.x < in.worldBboxMin.x ||
                next.pose.y < in.worldBboxMin.y ||
                next.pose.x > in.worldBboxMax.x ||
                next.pose.y > in.worldBboxMax.y)
            {
                out.clear();
                return out;
            }

            RsShotEdge e;
            e.edge.ptgDist              = dist;
            e.edge.ptgIndex             = m.ptgIndex;
            e.edge.ptgPathIndex         = k;
            e.edge.ptgTrimmableSpeed    = 1.0;
            e.edge.ptgFinalGoalRelSpeed = 0;
            e.edge.ptgFinalRelativeGoal =
                goal.asSE2KinState().pose - state.pose;
            e.edge.stateFrom    = state;
            e.edge.stateTo      = next;
            e.edge.ptgStepIndex = *step;
            // Always interpolated (also sets estimatedExecTime): cost
            // evaluators and refine_trajectory() need it, and shots are few.
            edge_interpolated_path(
                e.edge, trs, ptg.getPathPose(k, *step), *step,
                params_.pathInterpolatedSegments);
            e.edge.cost = cost_path_segment(e.edge);
            e.stateTo   = next;
            out.push_back(e);
            state = next;
        }
    }
    // The chain ends at the goal, up to the time discretization of the PTG
    // trajectories. (A same-cell test would reject shots to goals on a cell
    // boundary.)
    const auto   goalPose = goal.asSE2KinState().pose;
    const double errXY =
        std::hypot(state.pose.x - goalPose.x, state.pose.y - goalPose.y);
    const double errPhi =
        goal.state.isPose()
            ? std::abs(mrpt::math::angDistance(state.pose.phi, goalPose.phi))
            : 0.0;
    if (errXY > 0.5 * params_.grid_resolution_xy ||
        errPhi > 0.5 * params_.grid_resolution_yaw)
    {
        out.clear();
    }
    return out;
}
