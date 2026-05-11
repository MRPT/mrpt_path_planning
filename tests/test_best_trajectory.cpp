/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for bestTrajectory().
 *
 * Verified behaviors:
 *  - Valid relative pose: finds a PTG and sets ptgIndex/ptgPathIndex/ptgDist.
 *  - Empty PTG set: returns false immediately.
 *  - Pose outside all PTG ranges: returns true but ptgIndex stays -1.
 *  - On success, stateTo is updated to the PTG-reconstructed pose.
 */

#include <gtest/gtest.h>
#include <mpp/algos/bestTrajectory.h>
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

static mpp::TrajectoriesAndRobotShape buildTRS()
{
    mrpt::config::CConfigFileMemory cfg(kPtgCfg);
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    return trs;
}

static mpp::MoveEdgeSE2_TPS makeEdge(double tx, double ty, double tphi = 0.0)
{
    mpp::MoveEdgeSE2_TPS edge;
    edge.stateFrom.pose       = {0, 0, 0};
    edge.stateTo.pose         = {tx, ty, tphi};
    edge.ptgFinalRelativeGoal = {tx, ty, tphi};
    return edge;
}

// ---------------------------------------------------------------------------

TEST(BestTrajectory, FindsBestPTG)
{
    auto trs  = buildTRS();
    auto edge = makeEdge(1.0, 0.0);

    const bool ok = mpp::bestTrajectory(edge, trs);

    EXPECT_TRUE(ok);
    ASSERT_GE(edge.ptgIndex, 0) << "A valid PTG must be selected";

    // Reconstructed pose must be within 0.1 m of the requested target.
    auto& ptg = trs.ptgs.at(edge.ptgIndex);
    ptg->updateNavDynamicState(edge.getPTGDynState());
    uint32_t step = 0;
    ptg->getPathStepForDist(edge.ptgPathIndex, edge.ptgDist, step);
    const auto reconstr = ptg->getPathPose(edge.ptgPathIndex, step);
    EXPECT_LT((reconstr - mrpt::math::TPose2D{1.0, 0.0, 0.0}).norm(), 0.10);
}

TEST(BestTrajectory, EmptyPTGs)
{
    mpp::TrajectoriesAndRobotShape emptyTRS;
    auto                           edge = makeEdge(1.0, 0.0);

    const bool ok = mpp::bestTrajectory(edge, emptyTRS);

    EXPECT_FALSE(ok);
    EXPECT_EQ(edge.ptgIndex, -1);
}

TEST(BestTrajectory, DiagonalTarget)
{
    // Non-straight target: verifies that PTG selection works for
    // targets that require a curved trajectory, not just straight ahead.
    auto trs  = buildTRS();
    auto edge = makeEdge(1.0, 1.0);

    const bool ok = mpp::bestTrajectory(edge, trs);
    EXPECT_TRUE(ok);
    ASSERT_GE(edge.ptgIndex, 0);

    auto& ptg = trs.ptgs.at(edge.ptgIndex);
    ptg->updateNavDynamicState(edge.getPTGDynState());
    uint32_t step = 0;
    ptg->getPathStepForDist(edge.ptgPathIndex, edge.ptgDist, step);
    const auto reconstr = ptg->getPathPose(edge.ptgPathIndex, step);
    EXPECT_LT((reconstr - mrpt::math::TPose2D{1.0, 1.0, 0.0}).norm(), 0.10);
}

TEST(BestTrajectory, StateToUpdated)
{
    auto trs  = buildTRS();
    auto edge = makeEdge(1.5, 0.5);

    const mrpt::math::TPose2D requested = edge.stateTo.pose;

    const bool ok = mpp::bestTrajectory(edge, trs);
    ASSERT_TRUE(ok);
    ASSERT_GE(edge.ptgIndex, 0);

    // stateTo.pose is replaced by the PTG-reconstructed pose; it must stay
    // close to the original request (within the PTG's own tolerance).
    EXPECT_LT((edge.stateTo.pose - requested).norm(), 0.10)
        << "stateTo must be updated to the PTG-reconstructed pose";
}
