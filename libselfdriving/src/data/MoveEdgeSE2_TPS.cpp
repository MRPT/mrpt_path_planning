/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/data/MoveEdgeSE2_TPS.h>

#include <sstream>

using namespace selfdriving;

ptg_t::TNavDynamicState MoveEdgeSE2_TPS::getPTGDynState() const
{
    mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState newDyn;

    newDyn.relTarget   = stateTo.pose - stateFrom.pose;
    newDyn.curVelLocal = stateFrom.vel;
    // Global to local velocity:
    newDyn.curVelLocal.rotate(-stateFrom.pose.phi);

    newDyn.targetRelSpeed = targetRelSpeed;

    return newDyn;
}

std::string MoveEdgeSE2_TPS::asString() const
{
    using namespace std::string_literals;

    std::stringstream ss;
    ss << "MoveEdgeSE2_TPS:\n"
          " - parentId: "
       << parentId
       << "\n"
          " - from: "
       << stateFrom.asString()
       << "\n"
          " - to: "
       << stateTo.asString()
       << "\n"
          " - {cost: "
       << cost
       << ", "
          "ptgIndex: "
       << static_cast<int>(ptgIndex)
       << ", "
          "ptgPathIndex: "
       << ptgPathIndex
       << ", "
          "ptgDist: "
       << ptgDist
       << ", "
          "targetRelSpeed: "
       << targetRelSpeed
       << ", "
          "estimatedExecTime: "
       << estimatedExecTime << "}\n"
       << "- interpolatedPath size: " << interpolatedPath.size() << "\n";

    return ss.str();
}
