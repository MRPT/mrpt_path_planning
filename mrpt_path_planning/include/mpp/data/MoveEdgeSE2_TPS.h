/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/data/SE2_KinState.h>
#include <mpp/data/basic_types.h>
#include <mpp/data/ptg_t.h>
#include <mrpt/graphs/TNodeID.h>

#include <cstdint>

namespace mpp
{
/** An edge for the move tree used for planning in SE2 and TP-space */
struct MoveEdgeSE2_TPS
{
    MoveEdgeSE2_TPS()  = default;
    ~MoveEdgeSE2_TPS() = default;

    /** The ID of the parent node in the tree (all edges must have a valid
     * starting node). */
    mrpt::graphs::TNodeID parentId = mrpt::graphs::INVALID_NODEID;

    SE2_KinState stateFrom, stateTo;

    /** cost associated to each motion, this should be defined by the user
     * according to a specific cost function */
    double cost = std::numeric_limits<double>::max();

    /** indicate the type of trajectory used for this motion */
    int8_t ptgIndex = -1;

    /** identify the trajectory number K of the type ptgIndex */
    int16_t ptgPathIndex = -1;

    /** identify the PTG "pseudometers" distance of the trajectory for this
     * motion segment (NOT normalized distances) */
    distance_t ptgDist = std::numeric_limits<distance_t>::max();

    normalized_speed_t ptgTrimmableSpeed = 1.0;

    mrpt::math::TPose2D ptgFinalRelativeGoal;
    normalized_speed_t  ptgFinalGoalRelSpeed = .0;

    duration_seconds_t estimatedExecTime = .0;

    ptg_t::TNavDynamicState getPTGDynState() const;

    /** Subsampled path, in coordinates relative to "stateFrom", stored here
     *  for rendering purposes, to avoid having to re-seed the PTG
     *  with the initial velocity state while visualization,
     *  and to estimate the pose at each time.
     *  Minimum length: 2=start and final pose.
     */
    std::map<duration_seconds_t, mrpt::math::TPose2D> interpolatedPath;

    /** For debugging purposes. */
    std::string asString() const;
};

}  // namespace mpp
