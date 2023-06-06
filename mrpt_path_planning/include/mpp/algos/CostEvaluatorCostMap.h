/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CPointsMap.h>

namespace mpp
{
/** Defines higher costs to paths that pass closer to obstacles.
 *
 */
class CostEvaluatorCostMap : public CostEvaluator
{
    DEFINE_MRPT_OBJECT(CostEvaluatorCostMap, mpp)

   public:
    CostEvaluatorCostMap() = default;
    ~CostEvaluatorCostMap();

    struct Parameters
    {
        Parameters();
        ~Parameters();

        static Parameters FromYAML(const mrpt::containers::yaml& c);

        double resolution                 = 0.05;  //!< [m]
        double preferredClearanceDistance = 0.4;  //!< [m]
        double maxCost                    = 2.0;

        /** If enabled, the cost of a path segment (MoveEdgeSE2_TPS) will be the
         * average of the set of pointwise evaluations along it.
         * Otherwise (default) the maximum cost (more conservative) will be kept
         * instead.
         */
        bool useAverageOfPath = false;

        /** If !=0, defines the costmap only around a given maximum squared ROI
         * around the current robot pose. Otherwise, the limits are given by the
         * obstacle points. */
        double maxRadiusFromRobot = .0;

        mrpt::containers::yaml as_yaml();
        void                   load_from_yaml(const mrpt::containers::yaml& c);
    };

    static CostEvaluatorCostMap::Ptr FromStaticPointObstacles(
        const mrpt::maps::CPointsMap&             obsPts,
        const Parameters&                         p            = Parameters(),
        const std::optional<mrpt::math::TPose2D>& curRobotPose = std::nullopt);

    /** Evaluate cost of move-tree edge */
    double operator()(const MoveEdgeSE2_TPS& edge) const override;

    mrpt::opengl::CSetOfObjects::Ptr get_visualization() const override;

    using cost_gridmap_t = mrpt::containers::CDynamicGrid<double>;

    const cost_gridmap_t cost_gridmap() const { return costmap_; }

    const Parameters& params() const { return params_; }

   private:
    cost_gridmap_t costmap_;
    Parameters     params_;

    double eval_single_pose(const mrpt::math::TPose2D& p) const;
};

}  // namespace mpp
