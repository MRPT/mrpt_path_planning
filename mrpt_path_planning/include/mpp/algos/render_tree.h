/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/data/RenderOptions.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <memory>

namespace mpp
{
/** Gets a CSetOfObjects::Ptr visual representation of a motion tree */
auto render_tree(
    const MotionPrimitivesTreeSE2& tree, const PlannerInput& pi,
    const RenderOptions& ro) -> std::shared_ptr<mrpt::opengl::CSetOfObjects>;

}  // namespace mpp
