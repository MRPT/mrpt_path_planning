/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MotionPrimitivesTree.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/data/trajectory_t.h>

#include <map>

namespace mpp
{
trajectory_t plan_to_trajectory(
    const MotionPrimitivesTreeSE2::edge_sequence_t& planEdges,
    const TrajectoriesAndRobotShape&                ptgInfo,
    const duration_seconds_t                        samplePeriod = 50e-3);

bool save_to_txt(const trajectory_t& traj, const std::string& fileName);

}  // namespace mpp
