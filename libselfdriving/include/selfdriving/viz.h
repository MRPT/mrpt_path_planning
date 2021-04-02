/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <selfdriving/PlannerOutput.h>
#include <selfdriving/RenderOptions.h>

namespace selfdriving
{
struct NavPlanRenderOptions
{
    /** dont return viz_nav_plan() until user closes the window */
    bool gui_modal = true;

    RenderOptions renderOptions;
};

void viz_nav_plan(
    const PlannerOutput&        plan,
    const NavPlanRenderOptions& opts = NavPlanRenderOptions());

}  // namespace selfdriving
