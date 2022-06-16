/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/opengl/opengl_frwds.h>
#include <selfdriving/data/MotionPrimitivesTree.h>
#include <selfdriving/data/PlannerInput.h>
#include <selfdriving/data/RenderOptions.h>

#include <memory>

namespace selfdriving
{
/** Gets a CSetOfObjects::Ptr visual representation of a motion tree */
auto render_tree(
    const MotionPrimitivesTreeSE2& tree, const PlannerInput& pi,
    const RenderOptions& ro) -> std::shared_ptr<mrpt::opengl::CSetOfObjects>;

}  // namespace selfdriving
