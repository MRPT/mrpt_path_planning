/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include "PlannerTypes.h"

namespace selfdrive
{
/** Finds the best trajectory between two kinematic states, given the set of
 * feasible trajectories.
 * \return true on success, false on no valid path found
 */
bool bestTrajectory(NavPlanAction& npa, const TrajectoriesAndRobotShape& trs);

}  // namespace selfdrive
