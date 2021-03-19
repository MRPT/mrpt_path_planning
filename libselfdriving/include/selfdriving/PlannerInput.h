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
    RobotShape                robot_shape;
    SE2_KinState              state_start, state_goal;
    mrpt::math::TPose2D       world_bbox_min, world_bbox_max;  //!< World bounds
    ObstacleSource::Ptr       obstacles;
    double                    min_step_len{0.25};  //!< Between waypoints [m]
    TrajectoriesAndRobotShape ptgs;
};

}  // namespace selfdriving
