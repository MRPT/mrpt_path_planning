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

namespace mpp
{
/** Penalizes path segments driven in reverse.
 *
 * Forward and reverse PTG trajectories at the same speed have the same travel
 * time, so a time-optimal planner has no reason to prefer driving forward and
 * may return paths that back up over long stretches. This evaluator adds
 * `reverseTimeCostFactor * estimatedExecTime` to every edge whose trajectory
 * moves backwards, so reversing is only used when it pays off (e.g. maneuvers
 * to reach a goal heading). The extra cost is non-negative, so admissible
 * heuristics remain admissible.
 *
 * Whether a trajectory moves backwards is determined from the sign of the
 * linear speed at its start, evaluated when each edge is evaluated (so it
 * accounts for the current dynamic state of PTGs whose motion depends on it),
 * with the PTGs given with setPTGs().
 */
class CostEvaluatorReverseMotion : public CostEvaluator
{
    DEFINE_MRPT_OBJECT(CostEvaluatorReverseMotion, mpp)

   public:
    struct Parameters
    {
        static Parameters FromYAML(const mrpt::containers::yaml& c);

        /** Extra cost per second of reverse motion (0: no penalty). */
        double reverseTimeCostFactor = 1.0;

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

    /** Whether trajectory `pathIndex` of PTG `ptgIndex` moves backwards, in
     * the current PTG dynamic state. */
    bool isReverse(int ptgIndex, int pathIndex) const;

   private:
    std::vector<std::shared_ptr<ptg_t>> ptgs_;
};

}  // namespace mpp
