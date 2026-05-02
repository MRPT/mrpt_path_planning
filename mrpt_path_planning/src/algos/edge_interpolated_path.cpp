/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/edge_interpolated_path.h>

void mpp::edge_interpolated_path(
    MoveEdgeSE2_TPS& edge, const TrajectoriesAndRobotShape& trs,
    const mrpt::math::TPose2D& reconstrRelPose,
    size_t                     ptg_step,
    const std::optional<size_t>&              numSegments)
{
    size_t nSeg = 0;

    if (numSegments.has_value()) { nSeg = *numSegments; }
    else
    {
        // Use same number than existing edge interpolated path:
        ASSERT_(edge.interpolatedPath.size() > 1);
        nSeg = edge.interpolatedPath.size();
    }

    auto& ptg = trs.ptgs.at(edge.ptgIndex);

    const duration_seconds_t dt = ptg->getPathStepDuration();

    auto& ip = edge.interpolatedPath;
    ip.clear();

    // t=0: fixed start relative pose
    // ---------------------------------
    ip[0 * dt] = {0, 0, 0};

    // interpolated path in between start and goal
    // ----------------------------------------------
    for (size_t i = 0; i < nSeg; i++)
    {
        const auto               iStep = ((i + 1) * ptg_step) / (nSeg + 2);
        const duration_seconds_t t     = iStep * dt;
        ip[t] = ptg->getPathPose(edge.ptgPathIndex, iStep);
    }

    // Final, known pose and time
    // ----------------------------------------------
    ip[ptg_step * dt] = reconstrRelPose;

    // Motion execution time:
    edge.estimatedExecTime = ptg_step * dt;
}
