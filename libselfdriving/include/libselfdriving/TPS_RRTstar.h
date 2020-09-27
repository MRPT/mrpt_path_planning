/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <libselfdriving/PlannerTypes.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

namespace selfdrive
{
class TPS_RRTstar : public mrpt::system::COutputLogger
{
   public:
    TPS_RRTstar();
    ~TPS_RRTstar() = default;

    NavPlan plan(const PlannerInput& in);

    struct Parameters
    {
        double grid_resolution = .05;  //!< [meters]
    };

    Parameters params_;

    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger profiler_{true, "TPS_RRTstar"};

   private:
};

}  // namespace selfdrive
