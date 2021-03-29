/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/graphs/TNodeID.h>
#include <selfdriving/SE2_KinState.h>
#include <selfdriving/ptg_t.h>

#include <cstdint>

namespace selfdriving
{
/** An edge for the move tree used for planning in SE2 and TP-space */
struct MoveEdgeSE2_TPS
{
    MoveEdgeSE2_TPS()  = default;
    ~MoveEdgeSE2_TPS() = default;

    /** The ID of the parent node in the tree (all edges must have a valid
     * starting node). */
    mrpt::graphs::TNodeID parentId = INVALID_NODEID;

    SE2_KinState stateFrom, stateTo;

    /** cost associated to each motion, this should be defined by the user
     * according to a specific cost function */
    double cost = std::numeric_limits<double>::max();

    /** indicate the type of trajectory used for this motion */
    int8_t ptgIndex = -1;

    /** identify the trajectory number K of the type ptgIndex */
    int16_t ptgPathIndex = -1;

    /** identify the PTG "pseudometers" distance of the trajectory for this
     * motion segment */
    double ptgDist = std::numeric_limits<double>::max();

    double targetRelSpeed = 1.0;

    double ptgSpeedScale     = 1.0;
    double estimatedExecTime = .0;

    ptg_t::TNavDynamicState getPTGDynState() const;

    /** Subsampled path, in coordinates relative to "stateFrom", stored here
     *  mainly for rendering purposes, and to avoid having to re-seed the PTG
     *  with the initial velocity state while visualization.
     */
    std::optional<std::vector<mrpt::math::TPose2D>> interpolatedPath;
};

}  // namespace selfdriving
