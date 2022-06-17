/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CPointsMap.h>
#include <selfdriving/algos/CostEvaluator.h>

namespace selfdriving
{
struct CostMapParameters
{
    double resolution = 0.05;  //!< [m]

    double preferredClearanceDistance = 0.5;  //!< [m]
    double maxCost                    = 100.0;

    /** If enabled, the cost of a path segment (MoveEdgeSE2_TPS) will be the
     * average of the set of pointwise evaluations along it.
     * Otherwise (default) the maximum cost (more conservative) will be kept
     * instead.
     */
    bool useAverageOfPath = false;
};

class CostEvaluatorCostMap : public CostEvaluator
{
    DEFINE_MRPT_OBJECT(CostEvaluatorCostMap, selfdriving)

   public:
    CostEvaluatorCostMap() = default;
    ~CostEvaluatorCostMap();

    static CostEvaluatorCostMap::Ptr FromStaticPointObstacles(
        const mrpt::maps::CPointsMap& obsPts,
        const CostMapParameters&      p = CostMapParameters());

    /** Evaluate cost of move-tree edge */
    double operator()(const MoveEdgeSE2_TPS& edge) const override;

    mrpt::opengl::CSetOfObjects::Ptr get_visualization() const override;

    using cost_gridmap_t = mrpt::containers::CDynamicGrid<double>;

    const cost_gridmap_t cost_gridmap() const { return costmap_; }

   private:
    cost_gridmap_t    costmap_;
    CostMapParameters params_;

    double eval_single_pose(const mrpt::math::TPose2D& p) const;
};

}  // namespace selfdriving
