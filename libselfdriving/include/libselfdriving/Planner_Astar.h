/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include "PlannerTypes.h"
#include "mrpt_1or2.h"

namespace selfdrive
{
class Planner_Astar
{
   public:
    Planner_Astar()  = default;
    ~Planner_Astar() = default;

    NavPlan plan(const PlannerInput& in);

    struct Parameters
    {
        double grid_resolution{.05};  //!< [meters]
    };

    Parameters params_;

    /** Time profiler (Default: enabled)*/
    CTimeLogger profiler_{true, "Planner_Astar"};

   private:
};

}  // namespace selfdrive
