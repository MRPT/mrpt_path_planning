/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/data/PlannerOutput.h>
#include <selfdriving/data/RenderOptions.h>
#include <selfdriving/data/trajectory_t.h>

namespace selfdriving
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
    const PlannerOutput& plan, const selfdriving::trajectory_t& traj,
    const RenderOptions&                  opts           = {},
    const std::vector<CostEvaluator::Ptr> costEvaluators = {});

}  // namespace selfdriving
