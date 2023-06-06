/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/data/RenderOptions.h>
#include <mpp/data/trajectory_t.h>

namespace mpp
{
struct VisualizationOptions
{
    /** dont return viz_nav_plan() until user closes the window */
    bool gui_modal = true;

    RenderOptions renderOptions;
};

void viz_nav_plan(
    const PlannerOutput& plan, const VisualizationOptions& opts = {},
    const std::vector<CostEvaluator::Ptr> costEvaluators = {});

void viz_nav_plan_animation(
    const PlannerOutput& plan, const mpp::trajectory_t& traj,
    const RenderOptions&                  opts           = {},
    const std::vector<CostEvaluator::Ptr> costEvaluators = {});

}  // namespace mpp
