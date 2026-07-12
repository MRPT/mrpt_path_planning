/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>

#include <optional>

namespace mpp
{
void edge_interpolated_path(
    MoveEdgeSE2_TPS& edge, const TrajectoriesAndRobotShape& trs,
    const mrpt::math::TPose2D& reconstrRelPose,
    size_t                     ptg_step,
    const std::optional<size_t>& numSegments = std::nullopt);

}
