/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <selfdriving/data/MotionPrimitivesTree.h>
#include <selfdriving/data/SE2_KinState.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>
#include <selfdriving/data/basic_types.h>

#include <map>

namespace selfdriving
{
using trajectory_t = std::map<duration_seconds_t, SE2_KinState>;

trajectory_t interpolate_path(
    const TrajectoriesAndRobotShape&                ptgInfo,
    const MotionPrimitivesTreeSE2::edge_sequence_t& edges,
    const duration_seconds_t                        samplePeriod = 50e-3);

}  // namespace selfdriving
