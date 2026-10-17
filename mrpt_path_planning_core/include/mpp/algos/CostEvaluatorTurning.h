/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mpp/algos/CostEvaluator.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/containers/yaml.h>

#include <vector>

/** Feature macro: mpp::CostEvaluatorTurning is available. */
#define MPP_HAS_TURNING_COST 1

namespace mpp
{
/** Penalizes turning: the heading change along path segments.
 *
 * A time-optimal planner sees almost no difference between driving straight
 * and following gentle arcs, so its paths may weave left and right along
 * open corridors, or include tight loops to adjust the heading. This
 * evaluator adds `turnCostFactor * |total heading change|` [rad] to every
 * edge, so straight motion is preferred unless turning pays off. The extra
 * cost is non-negative, so admissible heuristics remain admissible.
 *
 * The heading change is measured along the PTG trajectory of each edge (not
 * only between its end poses, so loops count in full), with the PTGs given
 * with setPTGs(), in their current dynamic state.
 */
class CostEvaluatorTurning : public CostEvaluator
{
    DEFINE_MRPT_OBJECT(CostEvaluatorTurning, mpp)

   public:
    struct Parameters
    {
        static Parameters FromYAML(const mrpt::containers::yaml& c);

        /** Extra cost per radian of heading change (0: no penalty). */
        double turnCostFactor = 1.0;

        mrpt::containers::yaml as_yaml() const;
        void                   load_from_yaml(const mrpt::containers::yaml& c);
    };

    /** Method parameters. Can be freely modified at any time after
     * construction. */
    Parameters params_;

    /** Must be called before evaluating edges, with the same PTGs used by the
     * planner. */
    void setPTGs(const TrajectoriesAndRobotShape& ptgs);

    /** Evaluate cost of move-tree edge */
    double operator()(const MoveEdgeSE2_TPS& edge) const override;

    /** Total absolute heading change [rad] along trajectory `pathIndex` of
     * PTG `ptgIndex`, up to step `step`, in the current PTG dynamic state. */
    double headingChange(int ptgIndex, int pathIndex, uint32_t step) const;

   private:
    std::vector<std::shared_ptr<ptg_t>> ptgs_;
};

}  // namespace mpp
