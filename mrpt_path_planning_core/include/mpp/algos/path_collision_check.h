/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/maps/CPointsMap.h>

#include <optional>
#include <vector>

/** Feature macro: mpp::first_colliding_edge() exists. */
#define MPP_HAS_PATH_COLLISION_CHECK 1

namespace mpp
{
/** Checks the edges of a planned path (in order, e.g. as returned by
 * MotionPrimitivesTree::backtrack_path()) against obstacles (global frame),
 * with the same PTG collision checks used while planning (for collision-grid
 * PTGs, the clearance is included). Null edges are ignored.
 *
 * Since the clearance is included, an edge leaving a pose closer than the
 * clearance to some obstacle is reported as colliding.
 *
 * As the planners, it changes the dynamic state of the PTGs in `trs` (users
 * set it before evaluating them): use a TrajectoriesAndRobotShape::clone() to
 * keep others from being affected, e.g. in other threads.
 *
 * \return The index in `edges` of the first edge that collides, or
 * std::nullopt if all of them are clear.
 */
std::optional<size_t> first_colliding_edge(
    const std::vector<const MoveEdgeSE2_TPS*>&      edges,
    const TrajectoriesAndRobotShape&                trs,
    const std::vector<mrpt::maps::CPointsMap::Ptr>& obstacles);

}  // namespace mpp
