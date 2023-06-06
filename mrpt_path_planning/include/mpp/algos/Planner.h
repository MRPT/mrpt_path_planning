/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/data/PlannerOutput.h>
#include <mpp/data/ProgressCallbackData.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <optional>
#include <vector>

namespace mpp
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

    mrpt::system::CTimeLogger& profiler_()
    {
        return customProfiler_ ? *customProfiler_ : defaultProfiler_;
    }

    void attachExternalProfiler_(mrpt::system::CTimeLogger& p)
    {
        customProfiler_ = &p;
    }

    cost_t cost_path_segment(const MoveEdgeSE2_TPS& edge) const;

    /** optional progress callback */
    planner_progress_callback_t progressCallback_;
    duration_seconds_t          progressCallbackCallPeriod_ = 0.1;

   private:
    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger  defaultProfiler_{true, "Planner"};
    mrpt::system::CTimeLogger* customProfiler_ = nullptr;
};

}  // namespace mpp
