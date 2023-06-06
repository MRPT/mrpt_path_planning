/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/bestTrajectory.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/core/get_env.h>

using namespace mpp;

bool mpp::bestTrajectory(
    MoveEdgeSE2_TPS& npa, const TrajectoriesAndRobotShape& trs,
    std::optional<mrpt::system::COutputLogger*> logger)
{
    MRPT_START

    const double HEADING_ERROR_WEIGHT = 1.0;

    npa.ptgIndex = -1;  // Default: invalid index

    if (trs.ptgs.empty()) return false;

    const auto relPose = npa.stateTo.pose - npa.stateFrom.pose;

    double best_ptg_target_dist = std::numeric_limits<double>::max();
    mrpt::math::TPose2D  best_ptg_relPose;
    mrpt::math::TTwist2D best_ptg_velAtEnd;

    // Make PTG realize of current kinematic state:
    const auto newDyn = npa.getPTGDynState();

    // For each ptg:
    for (unsigned int ptg_idx = 0; ptg_idx < trs.ptgs.size(); ptg_idx++)
    {
        auto& ptg = trs.ptgs[ptg_idx];

        ptg->updateNavDynamicState(newDyn);

        // Inverse map of relative pose:
        int    ptg_k;
        double ptg_norm_dist;
        if (!ptg->inverseMap_WS2TP(relPose.x, relPose.y, ptg_k, ptg_norm_dist))
        {
            if (logger)
            {
                logger.value()->logFmt(
                    mrpt::system::LVL_WARN, "ptg[%u] out of range relPose: %s",
                    ptg_idx, relPose.asString().c_str());
            }
            // Out of PTG range. Cannot do anything here.
            continue;
        }

        const double ptg_dist = ptg_norm_dist * ptg->getRefDistance();

        // Evaluate distance to target:
        MRPT_TODO("add other optimality criteria?");
        uint32_t ptg_step;
        ptg->getPathStepForDist(ptg_k, ptg_dist, ptg_step);

        const auto reconstr_pose = ptg->getPathPose(ptg_k, ptg_step);

        // Sanity check: (ignore heading since relPose may not have a valid
        // one)
        const auto reconstrErr = reconstr_pose - relPose;
        ASSERT_LT_(reconstrErr.norm(), 0.10);

        double this_ptg_dist_at_target =
            reconstrErr.norm() +
            std::abs(reconstrErr.phi) * HEADING_ERROR_WEIGHT;

        // Is this the best path segment so far?
        if (this_ptg_dist_at_target < best_ptg_target_dist)
        {
            best_ptg_target_dist = this_ptg_dist_at_target;

            npa.ptgIndex      = ptg_idx;
            npa.ptgPathIndex  = ptg_k;
            npa.ptgDist       = ptg_dist;
            best_ptg_relPose  = reconstr_pose;
            best_ptg_velAtEnd = ptg->getPathTwist(ptg_k, ptg_step);
        }

    }  // end for each ptg

    // Any valid PTG? Take goal target pose from PTG, since it represents
    // the predicted robot pose more accurately than the initial plan, which
    // may not take care of robot kinematics limitations:
    if (npa.ptgIndex >= 0)
    {
        if (logger)
        {
            logger.value()->logFmt(
                mrpt::system::LVL_DEBUG, "bestTrajectory(): before: %s",
                npa.stateTo.asString().c_str());
        }

        // Correct pose:
        npa.stateTo.pose = npa.stateFrom.pose + best_ptg_relPose;

        // Update vel:
        npa.stateTo.vel = best_ptg_velAtEnd;
        // local to global:
        npa.stateTo.vel.rotate(npa.stateTo.pose.phi);

        if (logger)
        {
            logger.value()->logFmt(
                mrpt::system::LVL_DEBUG, "bestTrajectory(): after: %s",
                npa.stateTo.asString().c_str());
        }
    }

    return true;
    MRPT_END
}
