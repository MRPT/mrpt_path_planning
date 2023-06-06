/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/data/basic_types.h>

#include <limits>
#include <vector>

namespace mpp
{
struct ProgressCallbackData
{
    ProgressCallbackData()  = default;
    ~ProgressCallbackData() = default;

    cost_t bestCostFromStart = std::numeric_limits<cost_t>::max();
    cost_t bestCostToGoal    = std::numeric_limits<cost_t>::max();
    MotionPrimitivesTreeSE2::edge_sequence_t bestPath;
    std::optional<TNodeID>                   bestFinalNode;
    const MotionPrimitivesTreeSE2*           tree              = nullptr;
    const PlannerInput*                      originalPlanInput = nullptr;
    const std::vector<CostEvaluator::Ptr>*   costEvaluators    = nullptr;
};

using planner_progress_callback_t =
    std::function<void(const ProgressCallbackData&)>;

}  // namespace mpp
