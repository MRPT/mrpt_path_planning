/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/algos/render_tree.h>
#include <selfdriving/algos/within_bbox.h>

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

TPS_Astar::TPS_Astar() : mrpt::system::COutputLogger("TPS_Astar") {}

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

    //  1  |  X_T ← {X_0 }    # Tree nodes (state space)
    // ------------------------------------------------------------------
    tree.root = tree.next_free_node_ID();
    tree.insert_root_node(tree.root, in.stateStart);

    //  2  |  E T ← ∅         # Tree edges
    // ------------------------------------------------------------------
    tree.edges_to_children.clear();

    const TNodeID goalNodeId = tree.next_free_node_ID();
    po.goalNodeId            = goalNodeId;

    // TODO: A*

    // A* ended, now collect the result:
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
