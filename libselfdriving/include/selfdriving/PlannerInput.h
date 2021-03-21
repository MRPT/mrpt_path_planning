/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <selfdriving/ObstacleSource.h>
#include <selfdriving/SE2_KinState.h>
#include <selfdriving/TrajectoriesAndRobotShape.h>

namespace selfdriving
{
struct PlannerInput
{
    SE2_KinState              stateStart, stateGoal;
    mrpt::math::TPose2D       worldBboxMin, worldBboxMax;  //!< World bounds
    ObstacleSource::Ptr       obstacles;
    double                    minStepLength = 0.15;  //!< Between waypoints [m]
    size_t                    maxPlanIterations = 100000;
    TrajectoriesAndRobotShape ptgs;
};

}  // namespace selfdriving
