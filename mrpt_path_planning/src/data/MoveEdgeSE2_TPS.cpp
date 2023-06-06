/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/MoveEdgeSE2_TPS.h>

#include <sstream>

using namespace mpp;

ptg_t::TNavDynamicState MoveEdgeSE2_TPS::getPTGDynState() const
{
    mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState newDyn;

    newDyn.curVelLocal = stateFrom.vel;
    // Global to local velocity:
    newDyn.curVelLocal.rotate(-stateFrom.pose.phi);

    newDyn.relTarget      = ptgFinalRelativeGoal;
    newDyn.targetRelSpeed = ptgFinalGoalRelSpeed;

    return newDyn;
}

std::string MoveEdgeSE2_TPS::asString() const
{
    using namespace std::string_literals;

    std::stringstream ss;
    ss << "-\n"
          "  parentId: "
       << parentId
       << "\n"
          "  from: "
       << stateFrom.asString()
       << "\n"
          "  to: "
       << stateTo.asString()
       << "\n"
          "  {cost: "
       << cost
       << ", "
          "ptgIndex: "
       << static_cast<int>(ptgIndex)
       << ", "
          "path: "
       << ptgPathIndex
       << ", "
          "ptgDist: "
       << ptgDist
       << ", "
          "trimSpeed: "
       << ptgTrimmableSpeed
       << ", "
          "finalRelGoal: "
       << ptgFinalRelativeGoal
       << ", "
          "finalGoalSpeed: "
       << ptgFinalGoalRelSpeed
       << ", "
          "estimatedExecTime: "
       << estimatedExecTime << "}\n"
       << "  interpolatedPathSize: " << interpolatedPath.size() << "\n";

    return ss.str();
}
