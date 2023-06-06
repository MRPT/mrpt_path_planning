/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/SE2_KinState.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/math/TPose2D.h>

#include <functional>

namespace mpp
{
struct PlannerInput
{
    SE2_KinState        stateStart;
    SE2orR2_KinState    stateGoal;
    mrpt::math::TPose2D worldBboxMin, worldBboxMax;  //!< World bounds
    std::vector<ObstacleSource::Ptr> obstacles;
    TrajectoriesAndRobotShape        ptgs;
};

}  // namespace mpp
