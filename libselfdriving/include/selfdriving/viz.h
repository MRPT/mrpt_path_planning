/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include "PlannerOutput.h"

namespace selfdriving
{
struct NavPlanRenderOptions
{
    /** dont return viz_nav_plan() until user closes the window */
    bool gui_modal                = true;
    int  show_robot_shape_every_N = 1;

    /** Render the PTG path segments between key poses */
    bool show_ptg_path_segments = true;
};

void viz_nav_plan(
    const PlannerOutput&        plan,
    const NavPlanRenderOptions& opts = NavPlanRenderOptions());

}  // namespace selfdriving
