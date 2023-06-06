/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose2D.h>

namespace mpp
{
/** Returns local obstacles as seen from a given pose, clipped to a maximum
 * distance. */
void transform_pc_square_clipping(
    const mrpt::maps::CPointsMap& inMap, const mrpt::poses::CPose2D& asSeenFrom,
    const double MAX_DIST_XY, mrpt::maps::CPointsMap& outMap,
    bool appendToOutMap = true);

}  // namespace mpp
