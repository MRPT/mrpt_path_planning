/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <selfdriving/data/MotionPrimitivesTree.h>  // TODO: refactor in smaller headers?

namespace selfdriving
{
/** Returns TPS-distance (pseudometers, not normalized) to obstacles.
 * ptg dynamic state must be updated by the caller.
 */
distance_t tp_obstacles_single_path(
    const trajectory_index_t      tp_space_k_direction,
    const mrpt::maps::CPointsMap& localObstacles, const ptg_t& ptg);

}  // namespace selfdriving
