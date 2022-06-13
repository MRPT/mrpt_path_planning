/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/data/PlannerInput.h>
#include <selfdriving/data/PlannerOutput.h>

#include <vector>

namespace selfdriving
{
class Planner : public mrpt::rtti::CObject,
                virtual public mrpt::system::COutputLogger
{
    DEFINE_VIRTUAL_MRPT_OBJECT(Planner)

   public:
    Planner() = default;
    ~Planner();

    virtual PlannerOutput           plan(const PlannerInput& in) = 0;
    std::vector<CostEvaluator::Ptr> costEvaluators_;

    virtual mrpt::containers::yaml params_as_yaml()                = 0;
    virtual void params_from_yaml(const mrpt::containers::yaml& c) = 0;

    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger profiler_{true, "Planner"};
};

}  // namespace selfdriving
