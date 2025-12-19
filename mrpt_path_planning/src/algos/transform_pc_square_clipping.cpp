/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/transform_pc_square_clipping.h>

void mpp::transform_pc_square_clipping(
    const mrpt::maps::CPointsMap& inMap, const mrpt::poses::CPose2D& asSeenFrom,
    const double MAX_DIST_XY, mrpt::maps::CPointsMap& outMap,
    bool appendToOutMap)
{
    const auto   obs_xs = inMap.getPointsBufferRef_x();
    const auto   obs_ys = inMap.getPointsBufferRef_y();
    const size_t nObs   = inMap.size();

    if (!appendToOutMap) { outMap.clear(); }
    // Prealloc mem for speed-up
    outMap.reserve(nObs);

    const mrpt::poses::CPose2D invPose = -asSeenFrom;
    // We can safely discard the rest of obstacles, since they cannot be
    // converted into TP-Obstacles anyway!

    for (size_t obs = 0; obs < nObs; obs++)
    {
        const double gx = obs_xs[obs], gy = obs_ys[obs];

        if (std::abs(gx - asSeenFrom.x()) > MAX_DIST_XY ||
            std::abs(gy - asSeenFrom.y()) > MAX_DIST_XY)
        {
            // ignore this obstacle: anyway, I don't know how to map it to
            // TP-Obs!
            continue;
        }

        double ox, oy;
        invPose.composePoint(gx, gy, ox, oy);

        outMap.insertPointFast(ox, oy, 0);
    }
}
