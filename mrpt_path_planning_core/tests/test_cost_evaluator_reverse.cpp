/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for CostEvaluatorReverseMotion.
 *
 * Verified behaviors:
 *  - Forward / reverse C-PTG trajectories are classified correctly.
 *  - Reverse edges cost factor * exec time; forward edges and invalid indices
 *    cost nothing.
 *  - In a plan to a goal right behind the robot, the penalty reduces the time
 *    spent driving in reverse.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CostEvaluatorReverseMotion.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mrpt/config/CConfigFileMemory.h>

namespace
{
const char* kCPtgForwardReverse = R"cfg(
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

mpp::PlannerInput makeInput()
{
    mrpt::config::CConfigFileMemory cfg(kCPtgForwardReverse);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    for (const auto& ptg : in.ptgs.ptgs) { ptg->initialize(); }
    return in;
}

// Total estimated time driven in reverse along the plan solution.
double planReverseTime(double factor)
{
    auto in            = makeInput();
    in.stateStart.pose = {0, 0, 0};
    // Goal right behind the start with the same heading:
    in.stateGoal.state = mrpt::math::TPose2D(-5.0, 0.0, 0.0);
    in.worldBboxMin    = {-10, -10, -M_PI};
    in.worldBboxMax    = {10, 10, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 15;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.0, 2.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 30.0;
    planner.params_.heuristic_epsilon               = 1.0;

    auto ev = mpp::CostEvaluatorReverseMotion::Create();
    ev->params_.reverseTimeCostFactor = factor;
    ev->setPTGs(in.ptgs);
    planner.costEvaluators_.push_back(ev);

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success);
    if (!out.goalNodeId) { return -1; }

    const auto [nodes, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    double tRev               = 0;
    for (const auto* e : edges)
    {
        if (e != nullptr && ev->isReverse(e->ptgIndex, e->ptgPathIndex))
        {
            tRev += e->estimatedExecTime;
        }
    }
    return tRev;
}
}  // namespace

TEST(CostEvaluatorReverseMotion, ClassifiesTrajectories)
{
    const auto                      in = makeInput();
    mpp::CostEvaluatorReverseMotion ev;
    ev.setPTGs(in.ptgs);

    for (int k = 0; k < 41; k++)
    {
        EXPECT_FALSE(ev.isReverse(0, k)) << "k=" << k;
        EXPECT_TRUE(ev.isReverse(1, k)) << "k=" << k;
    }
    EXPECT_FALSE(ev.isReverse(-1, 0));
    EXPECT_FALSE(ev.isReverse(1, -1));
    EXPECT_FALSE(ev.isReverse(2, 0));
    EXPECT_FALSE(ev.isReverse(1, 41));
}

TEST(CostEvaluatorReverseMotion, EdgeCost)
{
    const auto                      in = makeInput();
    mpp::CostEvaluatorReverseMotion ev;
    ev.params_.reverseTimeCostFactor = 2.0;
    ev.setPTGs(in.ptgs);

    mpp::MoveEdgeSE2_TPS e;
    e.estimatedExecTime = 1.5;
    e.ptgPathIndex      = 10;

    e.ptgIndex = 0;
    EXPECT_DOUBLE_EQ(ev(e), 0.0);

    e.ptgIndex = 1;
    EXPECT_DOUBLE_EQ(ev(e), 3.0);

    ev.params_.reverseTimeCostFactor = 0.0;
    EXPECT_DOUBLE_EQ(ev(e), 0.0);
}

TEST(CostEvaluatorReverseMotion, LessReverseInPlans)
{
    const double tRevFree      = planReverseTime(0.0);
    const double tRevPenalized = planReverseTime(5.0);
    ASSERT_GE(tRevFree, 0.0);
    ASSERT_GE(tRevPenalized, 0.0);

    // Without penalty, backing straight up is the time-optimal plan:
    EXPECT_GT(tRevFree, 3.0);
    EXPECT_LT(tRevPenalized, 0.5 * tRevFree);
}
