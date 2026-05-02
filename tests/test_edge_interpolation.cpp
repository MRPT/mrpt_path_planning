/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Tests for edge_interpolated_path().
 *
 * Verified behaviors:
 *  - First pose in interpolatedPath is the identity (origin).
 *  - Last pose matches the target relative pose.
 *  - Intermediate poses lie on the PTG trajectory.
 *  - estimatedExecTime is set correctly.
 */

#include <gtest/gtest.h>
#include <mpp/algos/edge_interpolated_path.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/config/CConfigFileMemory.h>

static const char* kPtgCfg = R"cfg(
[SelfDriving]
min_obstacles_height  = 0.0
max_obstacles_height  = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::HolonomicBlend
PTG0_refDistance = 5.0
PTG0_num_paths   = 61
PTG0_T_ramp_max  = 1.0
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_expr_V      = V_MAX * trimmable_speed
PTG0_expr_W      = W_MAX * trimmable_speed * min(1.0, 0.1+abs(dir)/(10*3.14159265/180))
PTG0_expr_T_ramp = T_ramp_max

RobotModel_circular_shape_radius = 0.15
)cfg";

TEST(EdgeInterpolation, IntermediatePointsOnPTG)
{
    mrpt::config::CConfigFileMemory cfg(kPtgCfg);
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");

    auto& ptg = trs.ptgs.at(0);

    // Pick a trajectory index and a step somewhere in the middle.
    const size_t k    = 30;   // trajectory index (0..60)
    const size_t step = 50;   // PTG step index

    const mrpt::math::TPose2D reconstrRelPose = ptg->getPathPose(k, step);
    const double              dt              = ptg->getPathStepDuration();

    // Build a minimal edge.
    mpp::MoveEdgeSE2_TPS edge;
    edge.ptgIndex     = 0;
    edge.ptgPathIndex = static_cast<int16_t>(k);
    edge.ptgDist      = ptg->getPathDist(k, step);

    // Pre-populate interpolatedPath so the numSegments=nullopt branch can
    // read its size; here we pass numSegments explicitly instead.
    const size_t nSeg = 5;
    mpp::edge_interpolated_path(edge, trs, reconstrRelPose, step, nSeg);

    const auto& ip = edge.interpolatedPath;

    // Must have at least start + nSeg intermediates + end entries.
    ASSERT_GE(ip.size(), 2u);

    // First entry: identity pose at t=0.
    const auto& firstPose = ip.begin()->second;
    EXPECT_NEAR(firstPose.x,   0.0, 1e-9);
    EXPECT_NEAR(firstPose.y,   0.0, 1e-9);
    EXPECT_NEAR(firstPose.phi, 0.0, 1e-9);

    // Last entry: must match reconstrRelPose.
    const auto& lastPose = ip.rbegin()->second;
    EXPECT_NEAR(lastPose.x,   reconstrRelPose.x,   1e-6);
    EXPECT_NEAR(lastPose.y,   reconstrRelPose.y,   1e-6);
    EXPECT_NEAR(lastPose.phi, reconstrRelPose.phi, 1e-6);

    // estimatedExecTime must equal step * dt.
    EXPECT_NEAR(edge.estimatedExecTime, step * dt, 1e-9);

    // Intermediate entries must match the PTG's own path poses at their steps.
    // They are stored at t = iStep * dt, and iStep = (i+1)*step / (nSeg+2).
    for (size_t i = 0; i < nSeg; i++)
    {
        const size_t iStep = ((i + 1) * step) / (nSeg + 2);
        const double t     = iStep * dt;

        const auto it = ip.find(t);
        ASSERT_NE(it, ip.end())
            << "Missing interpolated entry at t=" << t;

        const mrpt::math::TPose2D expected = ptg->getPathPose(k, iStep);
        EXPECT_NEAR(it->second.x,   expected.x,   1e-6) << "at i=" << i;
        EXPECT_NEAR(it->second.y,   expected.y,   1e-6) << "at i=" << i;
        EXPECT_NEAR(it->second.phi, expected.phi, 1e-6) << "at i=" << i;
    }
}
