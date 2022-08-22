/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <selfdriving/data/SE2_KinState.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>
#include <selfdriving/interfaces/ObstacleSource.h>

namespace selfdriving
{
struct PlannerInput
{
    SE2_KinState        stateStart;
    SE2orR2_KinState    stateGoal;
    mrpt::math::TPose2D worldBboxMin, worldBboxMax;  //!< World bounds
    std::vector<ObstacleSource::Ptr> obstacles;
    TrajectoriesAndRobotShape        ptgs;
};

}  // namespace selfdriving
