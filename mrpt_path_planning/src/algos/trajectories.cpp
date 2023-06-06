/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/trajectories.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>

#include <fstream>
#include <iostream>

using namespace mpp;

trajectory_t mpp::plan_to_trajectory(
    const MotionPrimitivesTreeSE2::edge_sequence_t& planEdges,
    const TrajectoriesAndRobotShape&                ptgInfo,
    const duration_seconds_t                        samplePeriod)
{
    ASSERT_(ptgInfo.initialized());
    ASSERT_GT_(samplePeriod, 0.);

    trajectory_t out;

    duration_seconds_t  currentEdgeStartTime = .0;
    mrpt::math::TPose2D endOfLastEdge        = mrpt::math::TPose2D::Identity();

    for (const auto& edge : planEdges)
    {
        auto&        ptg    = ptgInfo.ptgs.at(edge->ptgIndex);
        const double ptg_dt = ptg->getPathStepDuration();
        ASSERT_GT_(ptg_dt, 0.);

        ptg->updateNavDynamicState(edge->getPTGDynState());
        if (auto* ptgTrim = dynamic_cast<ptg::SpeedTrimmablePTG*>(ptg.get());
            ptgTrim)
            ptgTrim->trimmableSpeed_ = edge->ptgTrimmableSpeed;

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
            trajectory_state_t& ts = out[currentEdgeStartTime + step * ptg_dt];
            // populate it:
            ts.state.pose =
                endOfLastEdge + ptg->getPathPose(edge->ptgPathIndex, step);
            ts.state.vel = ptg->getPathTwist(edge->ptgPathIndex, step);

            ts.ptgIndex     = edge->ptgIndex;
            ts.ptgPathIndex = edge->ptgPathIndex;
            ts.ptgStep      = step;

            if (step == ptgFinalStep) break;
        }
        currentEdgeStartTime += ptgFinalStep * ptg_dt;
        endOfLastEdge = out.rbegin()->second.state.pose;
    }

    return out;
}

bool mpp::save_to_txt(const trajectory_t& traj, const std::string& fileName)
{
    std::ofstream f;
    f.open(fileName);
    if (!f.is_open()) return false;

    f << mrpt::format(
        "%% %15s  %15s %15s %15s  %15s %15s %15s"
        " %15s %15s %15s\n",  //
        "Time [s]",  //
        "x_global [m]", "y_global [m]", "phi [rad]",  //
        "vx_local [m]", "vy_local[m]", "omega [rad/s]",  //
        "PTG_index", "PTG_traj_index", "PTG_step");

    for (const auto& kv : traj)
    {
        const auto  timestamp = kv.first;
        const auto& ts        = kv.second;

        const auto& p  = ts.state.pose;
        const auto& tw = ts.state.vel;

        f << mrpt::format(
            "%15.03f %15.03f %15.03f %15.03f  %15.03f %15.03f %15.03f "
            "  %15u %15u %15u"
            "\n",
            timestamp,  //
            p.x, p.y, p.phi,  //
            tw.vx, tw.vy, tw.omega,  //
            static_cast<unsigned int>(ts.ptgIndex),
            static_cast<unsigned int>(ts.ptgPathIndex),
            static_cast<unsigned int>(ts.ptgStep));
    }

    return true;
}
