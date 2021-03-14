/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <selfdriving/PlannerInput.h>

namespace selfdriving
{
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

    /** Total cost of the best found path (cost; Euclidean distance) */
    double pathCost = std::numeric_limits<double>::max();

    /** The ID of the best target node in the tree */
    mrpt::graphs::TNodeID best_goal_node_id = INVALID_NODEID;

    /** The set of target nodes within an acceptable distance to target
     * (including `best_goal_node_id` and others) */
    std::set<mrpt::graphs::TNodeID> acceptable_goal_node_ids;

    /** The generated motion tree that explores free space starting at "start"
     */
    MotionPrimitivesTreeSE2 motionTree;
};

}  // namespace selfdriving
