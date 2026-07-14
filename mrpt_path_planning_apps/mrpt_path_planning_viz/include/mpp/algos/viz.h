/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/data/RenderOptions.h>
#include <mpp/data/trajectory_t.h>

#include <string>

namespace mpp
{
struct VisualizationOptions
{
    /** dont return viz_nav_plan() until user closes the window */
    bool gui_modal = true;

    /** Window title. If empty, a default title is used.
     * Only affects the non-modal case: repeated non-modal calls reuse a
     * single window instead of opening a new one each time, so this is
     * how callers can show, e.g., a per-request counter in the title. */
    std::string windowTitle;

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
