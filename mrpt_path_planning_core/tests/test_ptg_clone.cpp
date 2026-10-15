/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * PTG copies (TrajectoriesAndRobotShape::clone()) are independent PTGs:
 *  - A clone evaluates exactly as its original, also once the original is
 *    destroyed.
 *  - Changing the dynamic state of a clone does not affect the original.
 *  - Planners running concurrently, each with its own clone, give the same
 *    results as a single planner.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <memory>
#include <thread>
#include <vector>

namespace
{
const char* kDiffDriveCfg = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0

PTG_COUNT = 2

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 4.0
PTG0_num_paths   = 41
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0

PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.10
PTG1_refDistance = 4.0
PTG1_num_paths   = 41
PTG1_v_max_mps   = 1.0
PTG1_w_max_dps   = 60.0
PTG1_K           = -1.0

RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

const char* kHolonomicCfg = R"cfg(
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

mpp::TrajectoriesAndRobotShape loadPtgs(const char* cfgText)
{
    mrpt::config::CConfigFileMemory cfg(cfgText);
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    return trs;
}

// Compares the paths and TP-obstacles of two PTGs.
void expectSamePtg(const mpp::ptg_t& a, const mpp::ptg_t& b)
{
    ASSERT_EQ(a.getPathCount(), b.getPathCount());
    for (uint16_t k = 0; k < a.getPathCount(); k += 5)
    {
        const auto n = a.getPathStepCount(k);
        ASSERT_EQ(n, b.getPathStepCount(k));
        for (uint32_t step = 0; step < n; step += std::max<uint32_t>(1, n / 7))
        {
            const auto pa = a.getPathPose(k, step);
            const auto pb = b.getPathPose(k, step);
            EXPECT_DOUBLE_EQ(pa.x, pb.x);
            EXPECT_DOUBLE_EQ(pa.y, pb.y);
            EXPECT_DOUBLE_EQ(pa.phi, pb.phi);
        }
    }
    for (double ox = -2.0; ox <= 2.0; ox += 0.37)
    {
        for (double oy = -2.0; oy <= 2.0; oy += 0.41)
        {
            std::vector<double> ta;
            std::vector<double> tb;
            a.initTPObstacles(ta);
            b.initTPObstacles(tb);
            a.updateTPObstacle(ox, oy, ta);
            b.updateTPObstacle(ox, oy, tb);
            ASSERT_EQ(ta, tb);
        }
    }
}

void setStartVelocity(mpp::ptg_t& ptg, double vx)
{
    mpp::ptg_t::TNavDynamicState ds;
    ds.curVelLocal    = {vx, 0, 0};
    ds.relTarget      = {5.0, 0, 0};
    ds.targetRelSpeed = 1.0;
    ptg.updateNavDynamicState(ds);
}

}  // namespace

TEST(PtgClone, DiffDriveCloneEvaluatesAsOriginal)
{
    auto original = std::make_unique<mpp::TrajectoriesAndRobotShape>(
        loadPtgs(kDiffDriveCfg));
    const auto copy = original->clone();

    ASSERT_EQ(copy.ptgs.size(), original->ptgs.size());
    for (size_t i = 0; i < copy.ptgs.size(); i++)
    {
        EXPECT_NE(copy.ptgs[i].get(), original->ptgs[i].get());
        expectSamePtg(*copy.ptgs[i], *original->ptgs[i]);
    }

    // The copy keeps working once the original is gone:
    const auto reference = original->clone();
    original.reset();
    for (size_t i = 0; i < copy.ptgs.size(); i++)
    {
        expectSamePtg(*copy.ptgs[i], *reference.ptgs[i]);
    }
}

TEST(PtgClone, HolonomicCloneIsIndependent)
{
    const auto original = loadPtgs(kHolonomicCfg);
    setStartVelocity(*original.ptgs.at(0), 0.0);

    auto copy = original.clone();
    expectSamePtg(*copy.ptgs.at(0), *original.ptgs.at(0));

    // A different dynamic state in the copy changes its paths, not the
    // original ones:
    const auto before = original.ptgs.at(0)->getPathPose(10, 50);
    setStartVelocity(*copy.ptgs.at(0), 0.8);
    auto trim =
        std::dynamic_pointer_cast<mpp::ptg::SpeedTrimmablePTG>(copy.ptgs.at(0));
    ASSERT_TRUE(trim);
    trim->trimmableSpeed_ = 0.5;

    const auto after     = original.ptgs.at(0)->getPathPose(10, 50);
    const auto copyAfter = copy.ptgs.at(0)->getPathPose(10, 50);
    EXPECT_DOUBLE_EQ(before.x, after.x);
    EXPECT_DOUBLE_EQ(before.y, after.y);
    EXPECT_GT(
        std::abs(copyAfter.x - before.x) + std::abs(copyAfter.y - before.y),
        1e-3);
}

TEST(PtgClone, ConcurrentPlannersWithClonesMatchSingleRun)
{
    const auto ptgs = loadPtgs(kDiffDriveCfg);

    // A wall between start and goal, so the search expands many nodes:
    auto makeInput = [&](const mpp::TrajectoriesAndRobotShape& trs)
    {
        auto wall = mrpt::maps::CSimplePointsMap::Create();
        for (double y = -3.0; y <= 3.0; y += 0.05)
        {
            wall->insertPoint(2.0f, static_cast<float>(y), 0.0f);
        }
        mpp::PlannerInput in;
        in.ptgs            = trs;
        in.stateStart.pose = {0, 0, 0};
        in.stateGoal.state = mrpt::math::TPose2D{4.0, 0.0, 0.0};
        in.worldBboxMin    = {-3, -6, -M_PI};
        in.worldBboxMax    = {7, 6, M_PI};
        in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(wall));
        return in;
    };
    auto runPlan = [&](const mpp::TrajectoriesAndRobotShape& trs)
    {
        mpp::TPS_Astar planner;
        planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
        planner.params_.maximumComputationTime = 60.0;
        return planner.plan(makeInput(trs));
    };

    const auto reference = runPlan(ptgs.clone());
    ASSERT_TRUE(reference.success);

    constexpr size_t                            kThreads = 4;
    std::vector<mpp::PlannerOutput>             outputs(kThreads);
    std::vector<std::thread>                    threads;
    std::vector<mpp::TrajectoriesAndRobotShape> clones;
    for (size_t i = 0; i < kThreads; i++) { clones.push_back(ptgs.clone()); }
    for (size_t i = 0; i < kThreads; i++)
    {
        threads.emplace_back([&, i]() { outputs[i] = runPlan(clones[i]); });
    }
    for (auto& t : threads) { t.join(); }

    for (const auto& o : outputs)
    {
        EXPECT_TRUE(o.success);
        EXPECT_EQ(o.numExpandedNodes, reference.numExpandedNodes);
        EXPECT_DOUBLE_EQ(o.pathCost, reference.pathCost);
    }
}
