/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/initializer.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/interfaces/TargetApproachController.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>
#include <selfdriving/ptgs/HolonomicBlend.h>

MRPT_INITIALIZER(selfdriving_register)
{
    using namespace selfdriving;
    using mrpt::rtti::registerClass;

    // Costs:
    registerClass(CLASS_ID(CostEvaluator));
    registerClass(CLASS_ID(CostEvaluatorCostMap));
    registerClass(CLASS_ID(CostEvaluatorPreferredWaypoint));

    // Planners:
    registerClass(CLASS_ID(Planner));
    registerClass(CLASS_ID(TPS_Astar));

    // Interfaces:
    registerClass(CLASS_ID(VehicleMotionInterface));
    registerClass(CLASS_ID(TargetApproachController));

    // PTGs:
    registerClass(CLASS_ID(ptg::HolonomicBlend));
}
