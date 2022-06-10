/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/rtti/CObject.h>
#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/data/PlannerInput.h>
#include <selfdriving/data/PlannerOutput.h>

#include <vector>

namespace selfdriving
{

class Planner : public mrpt::rtti::CObject
{
    DEFINE_VIRTUAL_MRPT_OBJECT(Planner)

   public:
    Planner() = default;
    ~Planner();

    virtual PlannerOutput           plan(const PlannerInput& in) = 0;
    std::vector<CostEvaluator::Ptr> costEvaluators_;
};

}  // namespace selfdriving
