/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/graphs/TNodeID.h>
#include <selfdriving/data/MotionPrimitivesTree.h>
#include <selfdriving/data/PlannerInput.h>

#include <set>

namespace selfdriving
{
using TNodeID = mrpt::graphs::TNodeID;

/** The output of the path planner */
struct PlannerOutput
{
    PlannerOutput()  = default;
    ~PlannerOutput() = default;

    PlannerInput originalInput;

    bool success = false;

    /** Time spent (in secs) */
    double computationTime = 0;

    /** Distance from best found path to goal */
    double goalDistance = std::numeric_limits<double>::max();

    /** Total cost of the best found path.
     * Cost is the Euclidean distance, modified by the additional cost map
     * layers.
     */
    double pathCost = std::numeric_limits<double>::max();

    /** The tree node ID for the goal pose */
    TNodeID goalNodeId = mrpt::graphs::INVALID_NODEID;

    /** The tree node with the smallest cost-to-goal. This will be identical
     *  to goalNodeId for successful plans reaching the desired goal state, or
     *  something different for unfinished or unsuccessful plans.
     */
    TNodeID bestNodeId = mrpt::graphs::INVALID_NODEID;

    /** The generated motion tree that explores free space starting at "start"
     */
    MotionPrimitivesTreeSE2 motionTree;
};

}  // namespace selfdriving
