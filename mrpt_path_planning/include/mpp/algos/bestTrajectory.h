/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/system/COutputLogger.h>

#include <optional>

namespace mpp
{
/** Finds the best trajectory between two kinematic states, given the set of
 * feasible trajectories.
 * \return true on success, false on no valid path found
 */
bool bestTrajectory(
    MoveEdgeSE2_TPS& npa, const TrajectoriesAndRobotShape& trs,
    std::optional<mrpt::system::COutputLogger*> logger = std::nullopt);

}  // namespace mpp
