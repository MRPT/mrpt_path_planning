/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/MoveEdgeSE2_TPS.h>

using namespace selfdriving;

ptg_t::TNavDynamicState MoveEdgeSE2_TPS::getPTGDynState() const
{
    mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState newDyn;

    newDyn.relTarget   = stateTo.pose - stateFrom.pose;
    newDyn.curVelLocal = stateFrom.vel;
    // Global to local velocity:
    newDyn.curVelLocal.rotate(-stateFrom.pose.phi);

    MRPT_TODO("Support stop at final pose?");
    newDyn.targetRelSpeed = 1.0;

    return newDyn;
}
