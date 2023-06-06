/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CSimplePointsMap.h>

namespace mpp
{
/** Defines lower (negative) costs to paths that pass closer to one or more
 * "preferred waypoints" defined by 2D coordinates (x,y)
 */
class CostEvaluatorPreferredWaypoint : public CostEvaluator
{
    DEFINE_MRPT_OBJECT(CostEvaluatorPreferredWaypoint, mpp)

   public:
    CostEvaluatorPreferredWaypoint() = default;
    ~CostEvaluatorPreferredWaypoint();

    struct Parameters
    {
        Parameters();
        ~Parameters();

        static Parameters FromYAML(const mrpt::containers::yaml& c);

        double waypointInfluenceRadius = 1.5;  //!< [m]
        double costScale               = 10.0;

        /** If enabled (default), the cost of a path segment (MoveEdgeSE2_TPS)
         * will be the average of the set of pointwise evaluations along it.
         * Otherwise the maximum (negative) cost will be kept instead.
         */
        bool useAverageOfPath = true;

        mrpt::containers::yaml as_yaml();
        void                   load_from_yaml(const mrpt::containers::yaml& c);
    };

    /** Method parameters. Can be freely modified at any time after
     * construction. */
    Parameters params_;

    /** The preferred waypoints must be defined by means of this method. */
    void setPreferredWaypoints(const std::vector<mrpt::math::TPoint2D>& pts);

    /** Evaluate cost of move-tree edge */
    double operator()(const MoveEdgeSE2_TPS& edge) const override;

    mrpt::opengl::CSetOfObjects::Ptr get_visualization() const override;

    const Parameters& params() const { return params_; }

   private:
    double eval_single_pose(const mrpt::math::TPose2D& p) const;

    mrpt::maps::CSimplePointsMap waypoints_;
};

}  // namespace mpp
