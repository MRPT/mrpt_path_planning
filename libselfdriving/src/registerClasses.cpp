/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/initializer.h>
#include <selfdriving/CostEvaluatorCostMap.h>

MRPT_INITIALIZER(selfdriving_register)
{
    using namespace selfdriving;

    mrpt::rtti::registerClass(CLASS_ID(CostEvaluator));
    mrpt::rtti::registerClass(CLASS_ID(CostEvaluatorCostMap));
}
