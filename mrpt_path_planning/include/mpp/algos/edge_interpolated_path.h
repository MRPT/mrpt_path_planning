/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>

#include <optional>

namespace mpp
{
void edge_interpolated_path(
    MoveEdgeSE2_TPS& edge, const TrajectoriesAndRobotShape& trs,
    const std::optional<mrpt::math::TPose2D>& reconstrRelPoseOpt = std::nullopt,
    const std::optional<size_t>&              ptg_stepOpt        = std::nullopt,
    const std::optional<size_t>&              numSegments = std::nullopt);

}
