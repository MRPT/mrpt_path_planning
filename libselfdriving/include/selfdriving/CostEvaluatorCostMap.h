/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CPointsMap.h>
#include <selfdriving/CostEvaluator.h>

namespace selfdriving
{
struct CostMapParameters
{
    double resolution = 0.05;  //!< [m]

    double preferredClearanceDistance = 1.0;  //!< [m]
    double maxCost                    = 5.0;
};

class CostEvaluatorCostMap
{
   public:
    CostEvaluatorCostMap() = default;
    ~CostEvaluatorCostMap();

    static CostEvaluatorCostMap FromStaticPointObstacles(
        const mrpt::maps::CPointsMap& obsPts,
        const CostMapParameters&      p = CostMapParameters());

    using cost_gridmap_t = mrpt::containers::CDynamicGrid<double>;

    /** Evaluate cost of move-tree edge */
    double operator()(const MoveEdgeSE2_TPS& edge) const;

   private:
    cost_gridmap_t costmap_;

    double eval_single_pose(const mrpt::math::TPose2D& p) const;
};

}  // namespace selfdriving
