/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/bestTrajectory.h>
#include <mrpt/core/get_env.h>

static const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

using namespace selfdrive;

bool selfdrive::bestTrajectory(
    NavPlanAction& npa, const TrajectoriesAndRobotShape& trs)
{
    MRPT_START

    const double HEADING_ERROR_WEIGHT = 1.0;

    npa.ptg_index = -1;  // Default: invalid index

    if (trs.ptgs.empty()) return false;

    const auto relPose = npa.state_to.pose - npa.state_from.pose;

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
            std::cout << "ptg[" << ptg_idx
                      << "] out of range relPose:" << relPose.asString()
                      << "\n";

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
        ASSERT_BELOW_(reconstrErr.norm(), 0.10);

        double this_ptg_dist_at_target =
            reconstrErr.norm() +
            std::abs(reconstrErr.phi) * HEADING_ERROR_WEIGHT;

        // Is this the best path segment so far?
        if (this_ptg_dist_at_target < best_ptg_target_dist)
        {
            best_ptg_target_dist = this_ptg_dist_at_target;

            npa.ptg_index      = ptg_idx;
            npa.ptg_path_index = ptg_k;
            npa.ptg_to_d       = ptg_dist;
            npa.ptg_path_alpha = ptg->index2alpha(ptg_k);
            best_ptg_relPose   = reconstr_pose;
            best_ptg_velAtEnd  = ptg->getPathTwist(ptg_k, ptg_step);
        }

    }  // end for each ptg

    // Any valid PTG? Take goal target pose from PTG, since it represents the
    // predicted robot pose more accurately than the initial plan, which may not
    // take care of robot kinematics limitations:
    if (npa.ptg_index >= 0)
    {
        if (VERBOSE) std::cout << "before: " << npa.state_to.asString() << "\n";

        // Correct pose:
        npa.state_to.pose = npa.state_from.pose + best_ptg_relPose;

        // Update vel:
        npa.state_to.vel = best_ptg_velAtEnd;
        // local to global:
        npa.state_to.vel.rotate(npa.state_to.pose.phi);

        if (VERBOSE) std::cout << " after: " << npa.state_to.asString() << "\n";
    }

    return true;
    MRPT_END
}
