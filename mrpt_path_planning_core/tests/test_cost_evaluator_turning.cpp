/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for CostEvaluatorTurning, and for A* exploring the straight
 * trajectory.
 *
 * Verified behaviors:
 *  - The heading change of the straight trajectory is zero, that of the
 *    tightest arc is w_max * time (loops count in full); invalid indices cost
 *    nothing.
 *  - Edges cost factor * heading change.
 *  - A* includes the straight trajectory even if the evenly spaced subset of
 *    `max_ptg_trajectories_to_explore` misses it, and with the turning cost
 *    a plan along an open corridor turns much less.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CostEvaluatorTurning.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>

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

// Total heading change [rad] of the plan solution, along a corridor (walls at
// y=+-1.5) from (0,0,0) to (10,0,0). `turnCost` <= 0: no turning cost.
double planHeadingChange(double turnCost)
{
    auto in = makeInput();

    auto obs = mrpt::maps::CSimplePointsMap::Create();
    for (double x = -2.0; x <= 12.0; x += 0.05)
    {
        obs->insertPoint(x, -1.5, 0);
        obs->insertPoint(x, 1.5, 0);
    }
    in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPose2D(10.0, 0.0, 0.0);
    in.worldBboxMin    = {-3, -2, -M_PI};
    in.worldBboxMax    = {13, 2, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy  = 0.20;
    planner.params_.grid_resolution_yaw = 10.0 * M_PI / 180.0;
    // Even count out of 41 paths: the evenly spaced subset misses the
    // straight one (index 20):
    planner.params_.max_ptg_trajectories_to_explore = 10;
    planner.params_.ptg_sample_timestamps           = {1.0, 2.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 30.0;

    std::shared_ptr<mpp::CostEvaluatorTurning> ev;
    if (turnCost > 0)
    {
        ev                         = mpp::CostEvaluatorTurning::Create();
        ev->params_.turnCostFactor = turnCost;
        ev->setPTGs(in.ptgs);
        planner.costEvaluators_.push_back(ev);
    }

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success);
    if (!out.goalNodeId) { return -1; }

    const auto [nodes, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    double total              = 0;
    auto   it                 = nodes.begin();
    for (auto itNext = std::next(it); itNext != nodes.end(); ++it, ++itNext)
    {
        total +=
            std::abs(mrpt::math::wrapToPi(itNext->pose.phi - it->pose.phi));
    }
    return total;
}
}  // namespace

TEST(CostEvaluatorTurning, HeadingChange)
{
    const auto                in = makeInput();
    mpp::CostEvaluatorTurning ev;
    ev.setPTGs(in.ptgs);

    const auto& ptg      = *in.ptgs.ptgs.at(0);
    const auto  kAhead   = ptg.alpha2index(0.0);
    const auto  stepsFor = [&](double t) {
        return static_cast<uint32_t>(std::round(t / ptg.getPathStepDuration()));
    };

    // Straight ahead: no heading change.
    EXPECT_NEAR(ev.headingChange(0, kAhead, stepsFor(2.0)), 0.0, 1e-6);

    // Tightest arc (60 deg/s) for 2 s, within the path length:
    const uint32_t steps =
        std::min<uint32_t>(stepsFor(2.0), ptg.getPathStepCount(0) - 1);
    const double t = steps * ptg.getPathStepDuration();
    EXPECT_NEAR(
        ev.headingChange(0, 0, steps), mrpt::DEG2RAD(60.0) * t,
        mrpt::DEG2RAD(3.0));

    // Invalid indices:
    EXPECT_DOUBLE_EQ(ev.headingChange(-1, 0, 10), 0.0);
    EXPECT_DOUBLE_EQ(ev.headingChange(2, 0, 10), 0.0);
    EXPECT_DOUBLE_EQ(ev.headingChange(0, -1, 10), 0.0);
    EXPECT_DOUBLE_EQ(ev.headingChange(0, 41, 10), 0.0);
}

TEST(CostEvaluatorTurning, EdgeCost)
{
    const auto                in = makeInput();
    mpp::CostEvaluatorTurning ev;
    ev.params_.turnCostFactor = 2.0;
    ev.setPTGs(in.ptgs);

    mpp::MoveEdgeSE2_TPS e;
    e.ptgIndex     = 0;
    e.ptgPathIndex = 0;
    e.ptgStepIndex = 100;

    const double dphi = ev.headingChange(0, 0, 100);
    EXPECT_GT(dphi, 0.0);
    EXPECT_DOUBLE_EQ(ev(e), 2.0 * dphi);

    ev.params_.turnCostFactor = 0.0;
    EXPECT_DOUBLE_EQ(ev(e), 0.0);
}

TEST(CostEvaluatorTurning, StraighterPlans)
{
    const double free      = planHeadingChange(0.0);
    const double penalized = planHeadingChange(1.0);
    ASSERT_GE(free, 0.0);
    ASSERT_GE(penalized, 0.0);

    // Driving straight is possible (the straight trajectory is explored) and
    // preferred with the turning cost:
    EXPECT_LT(penalized, mrpt::DEG2RAD(10.0));
    EXPECT_LE(penalized, free);
}
