/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <selfdriving/algos/CostEvaluator.h>
#include <selfdriving/algos/Planner.h>

namespace selfdriving
{
struct TPS_Astar_Parameters
{
    TPS_Astar_Parameters() = default;
    static TPS_Astar_Parameters FromYAML(const mrpt::containers::yaml& c);

    double SE2_metricAngleWeight = 1.0;

    /** Required to smooth interpolation of rendered paths, evaluation of
     * path cost, etc. */
    size_t pathInterpolatedSegments = 5;

    /** 0:disabled */
    size_t saveDebugVisualizationDecimation = 0;

    mrpt::containers::yaml as_yaml();
    void                   load_from_yaml(const mrpt::containers::yaml& c);
};

/**
 * Uses a SE(2) lattice to run an A* algorithm to find a kinematicaly feasible
 * path from "A" to "B" using a set of trajectories in the form of PTGs.
 *
 */
class TPS_Astar : public mrpt::system::COutputLogger, public Planner
{
    DEFINE_MRPT_OBJECT(TPS_Astar, selfdriving)

   public:
    TPS_Astar();
    virtual ~TPS_Astar() = default;

    PlannerOutput plan(const PlannerInput& in) override;

    TPS_Astar_Parameters params_;

    /** Time profiler (Default: enabled)*/
    mrpt::system::CTimeLogger profiler_{true, "TPS_Astar"};
};

}  // namespace selfdriving
