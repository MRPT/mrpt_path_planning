/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/interfaces/TargetApproachController.h>
#include <mpp/interfaces/VehicleMotionInterface.h>
#include <mpp/ptgs/DiffDrive_C.h>
#include <mpp/ptgs/HolonomicBlend.h>
#include <mrpt/core/initializer.h>

MRPT_INITIALIZER(selfdriving_register)
{
    using namespace mpp;
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
    registerClass(CLASS_ID(ptg::DiffDrive_C));
}
