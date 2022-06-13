/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/initializer.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/algos/TPS_RRTstar.h>

MRPT_INITIALIZER(selfdriving_register)
{
    using namespace selfdriving;

    // Costs:
    mrpt::rtti::registerClass(CLASS_ID(CostEvaluator));
    mrpt::rtti::registerClass(CLASS_ID(CostEvaluatorCostMap));

    // Planners:
    mrpt::rtti::registerClass(CLASS_ID(Planner));
    mrpt::rtti::registerClass(CLASS_ID(TPS_Astar));
    mrpt::rtti::registerClass(CLASS_ID(TPS_RRTstar));
}
