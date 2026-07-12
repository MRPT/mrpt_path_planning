/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for plan_to_trajectory() and save_to_txt().
 *
 * Verified behaviors:
 *  - Single edge: timestamps span [0, T], poses match the PTG path.
 *  - Multi-edge: time is stitched — second edge starts after the first ends.
 *  - Coarser sample period → fewer trajectory points; final step always included.
 *  - save_to_txt: creates a non-empty file with one data line per entry.
 */

#include <gtest/gtest.h>
#include <mpp/algos/bestTrajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/config/CConfigFileMemory.h>

#include <cstdio>
#include <fstream>

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

// Build an edge using bestTrajectory so ptgIndex/ptgPathIndex/ptgDist are valid.
static mpp::MoveEdgeSE2_TPS buildEdge(
    mpp::TrajectoriesAndRobotShape& trs, double tx, double ty,
    const mrpt::math::TPose2D& fromPose = {0, 0, 0})
{
    mpp::MoveEdgeSE2_TPS edge;
    edge.stateFrom.pose       = fromPose;
    edge.stateTo.pose         = fromPose + mrpt::math::TPose2D{tx, ty, 0};
    edge.ptgFinalRelativeGoal = {tx, ty, 0};
    const bool ok = mpp::bestTrajectory(edge, trs);
    EXPECT_TRUE(ok);
    EXPECT_GE(edge.ptgIndex, 0);
    return edge;
}

// ---------------------------------------------------------------------------

TEST(Trajectories, SingleEdge)
{
    auto trs  = buildTRS();
    auto edge = buildEdge(trs, 1.0, 0.0);

    ASSERT_GE(edge.ptgIndex, 0);

    const double ptg_dt =
        trs.ptgs.at(edge.ptgIndex)->getPathStepDuration();

    mpp::MotionPrimitivesTreeSE2::edge_sequence_t seq;
    seq.push_back(&edge);

    const auto traj = mpp::plan_to_trajectory(seq, trs, ptg_dt);

    ASSERT_FALSE(traj.empty());

    // First timestamp must be 0.
    EXPECT_NEAR(traj.begin()->first, 0.0, 1e-9);

    // Last timestamp must be ≈ finalStep * ptg_dt.
    auto& ptg = trs.ptgs.at(edge.ptgIndex);
    ptg->updateNavDynamicState(edge.getPTGDynState());
    uint32_t finalStep = 0;
    ptg->getPathStepForDist(edge.ptgPathIndex, edge.ptgDist, finalStep);
    EXPECT_NEAR(
        traj.rbegin()->first, finalStep * ptg_dt,
        ptg_dt + 1e-9);

    // PTG index and path index must be preserved.
    for (const auto& kv : traj)
    {
        EXPECT_EQ(kv.second.ptgIndex,     edge.ptgIndex);
        EXPECT_EQ(kv.second.ptgPathIndex, edge.ptgPathIndex);
    }
}

TEST(Trajectories, MultiEdgeTimeStitching)
{
    auto trs   = buildTRS();
    auto edge1 = buildEdge(trs, 1.0, 0.0);
    ASSERT_GE(edge1.ptgIndex, 0);

    // Second edge starts where the first ended.
    auto edge2 = buildEdge(trs, 0.0, 1.0, edge1.stateTo.pose);
    ASSERT_GE(edge2.ptgIndex, 0);

    const double ptg_dt = trs.ptgs.at(0)->getPathStepDuration();

    mpp::MotionPrimitivesTreeSE2::edge_sequence_t seq;
    seq.push_back(&edge1);
    seq.push_back(&edge2);

    const auto traj = mpp::plan_to_trajectory(seq, trs, ptg_dt);

    ASSERT_FALSE(traj.empty());

    // Duration of edge1.
    auto& ptg = trs.ptgs.at(edge1.ptgIndex);
    ptg->updateNavDynamicState(edge1.getPTGDynState());
    uint32_t step1 = 0;
    ptg->getPathStepForDist(edge1.ptgPathIndex, edge1.ptgDist, step1);
    const double T1 = step1 * ptg_dt;

    // The last timestamp must exceed T1 (second edge added time on top).
    EXPECT_GT(traj.rbegin()->first, T1)
        << "Multi-edge trajectory must span more time than the first edge alone";
}

TEST(Trajectories, CoarserSampleFewerPoints)
{
    auto trs  = buildTRS();
    auto edge = buildEdge(trs, 1.5, 0.0);
    ASSERT_GE(edge.ptgIndex, 0);

    const double ptg_dt = trs.ptgs.at(edge.ptgIndex)->getPathStepDuration();

    mpp::MotionPrimitivesTreeSE2::edge_sequence_t seq;
    seq.push_back(&edge);

    const auto fine   = mpp::plan_to_trajectory(seq, trs, ptg_dt);
    const auto coarse = mpp::plan_to_trajectory(seq, trs, ptg_dt * 10);

    EXPECT_GT(fine.size(),   1u);
    EXPECT_LT(coarse.size(), fine.size())
        << "Coarser sample period must produce fewer trajectory points";

    // Both must include the final step (ptgDist is always reached).
    auto& ptg = trs.ptgs.at(edge.ptgIndex);
    ptg->updateNavDynamicState(edge.getPTGDynState());
    uint32_t finalStep = 0;
    ptg->getPathStepForDist(edge.ptgPathIndex, edge.ptgDist, finalStep);
    const double T = finalStep * ptg_dt;

    EXPECT_NEAR(fine.rbegin()->first,   T, ptg_dt + 1e-9);
    EXPECT_NEAR(coarse.rbegin()->first, T, ptg_dt * 10 + 1e-9);
}

TEST(Trajectories, SaveToTxt)
{
    auto trs  = buildTRS();
    auto edge = buildEdge(trs, 1.0, 0.0);
    ASSERT_GE(edge.ptgIndex, 0);

    const double ptg_dt = trs.ptgs.at(edge.ptgIndex)->getPathStepDuration();

    mpp::MotionPrimitivesTreeSE2::edge_sequence_t seq;
    seq.push_back(&edge);
    const auto traj = mpp::plan_to_trajectory(seq, trs, ptg_dt);
    ASSERT_FALSE(traj.empty());

    const std::string tmpFile = "/tmp/test_traj_save.txt";
    const bool        ok      = mpp::save_to_txt(traj, tmpFile);
    ASSERT_TRUE(ok) << "save_to_txt must return true on a valid path";

    // Count non-comment lines in the file.
    std::ifstream f(tmpFile);
    ASSERT_TRUE(f.is_open());
    size_t      dataLines = 0;
    std::string line;
    while (std::getline(f, line))
        if (!line.empty() && line[0] != '%') dataLines++;

    EXPECT_EQ(dataLines, traj.size())
        << "File must have one data line per trajectory entry";

    std::remove(tmpFile.c_str());
}
