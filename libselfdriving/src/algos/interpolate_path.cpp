/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/algos/interpolate_path.h>

#include <iostream>

using namespace selfdriving;

trajectory_t selfdriving::interpolate_path(
    const TrajectoriesAndRobotShape&                ptgInfo,
    const MotionPrimitivesTreeSE2::edge_sequence_t& edges,
    const duration_seconds_t                        samplePeriod)
{
    ASSERT_(ptgInfo.initialized());
    ASSERT_GT_(samplePeriod, 0.);

    trajectory_t out;

    duration_seconds_t currentEdgeStartTime = .0;

    for (const auto& edge : edges)
    {
        auto&        ptg    = ptgInfo.ptgs.at(edge->ptgIndex);
        const double ptg_dt = ptg->getPathStepDuration();
        ASSERT_GT_(ptg_dt, 0.);

        ptg->updateNavDynamicState(edge->getPTGDynState());

        uint32_t ptgFinalStep = 0;
        bool     ok           = ptg->getPathStepForDist(
            edge->ptgPathIndex, edge->ptgDist, ptgFinalStep);
        ASSERT_(ok);
        uint32_t stepIncr =
            std::max<uint32_t>(1, mrpt::round(samplePeriod / ptg_dt));

        for (uint32_t step = 0;; step += stepIncr)
        {
            if (step > ptgFinalStep) step = ptgFinalStep;

            // Create path entry:
            SE2_KinState& state = out[currentEdgeStartTime + step * ptg_dt];
            // populate it:
            state.pose = ptg->getPathPose(edge->ptgPathIndex, step);
            state.vel  = ptg->getPathTwist(edge->ptgPathIndex, step);

            if (step == ptgFinalStep) break;
        }
        currentEdgeStartTime += ptgFinalStep * ptg_dt;
    }

    return out;
}
