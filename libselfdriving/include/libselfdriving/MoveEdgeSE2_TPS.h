/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <libselfdriving/SE2_KinState.h>
#include <libselfdriving/ptg_t.h>
#include <mrpt/graphs/TNodeID.h>
#include <cstdint>

namespace selfdrive
{
/** An edge for the move tree used for planning in SE2 and TP-space */
struct TMoveEdgeSE2_TPS
{
    /** The ID of the parent node in the tree (all edges must have a valid
     * starting node). */
    mrpt::graphs::TNodeID parentId = INVALID_NODEID;

    SE2_KinState stateFrom, stateTo;

    /** cost associated to each motion, this should be defined by the user
     * according to a specific cost function */
    double cost = std::numeric_limits<double>::max();

    /** indicate the type of trajectory used for this motion */
    int8_t ptgIndex = -1;

    /** identify the trajectory number K of the type ptg_index */
    int16_t ptgK = -1;

    /** identify the PTG normalized distance [0,1] of the trajectory for this
     * motion segment */
    double ptgDist = std::numeric_limits<double>::max();

    double ptgSpeedScale     = 1.0;
    double estimatedExecTime = .0;

    ptg_t::TNavDynamicState getPTGDynState() const;

    TMoveEdgeSE2_TPS() = default;
};

}  // namespace selfdrive
