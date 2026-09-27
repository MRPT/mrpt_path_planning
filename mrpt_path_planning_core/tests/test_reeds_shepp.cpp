/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for the Reeds-Shepp distance and the Reeds-Shepp heuristic of
 * TPS_Astar.
 *
 * Verified behaviors:
 *  - Closed-form values (straight forward/backward, quarter turn).
 *  - Metric properties (symmetry, triangle inequality).
 *  - Lower bound of the length of every DiffDrive_C trajectory.
 *  - In a planned tree with a full-pose goal: consistency along every edge
 *    and admissibility at the start.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/reeds_shepp.h>
#include <mpp/data/PlannerInput.h>
#include <mrpt/config/CConfigFileMemory.h>

#include <random>

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

// v_max / w_max of the PTGs above:
const double kTurningRadius = 1.0 / (60.0 * M_PI / 180.0);
}  // namespace

TEST(ReedsShepp, ClosedFormValues)
{
    const double R = 2.0;
    EXPECT_NEAR(mpp::reeds_shepp_distance({0, 0, 0}, {5, 0, 0}, R), 5.0, 1e-9);
    EXPECT_NEAR(mpp::reeds_shepp_distance({0, 0, 0}, {-5, 0, 0}, R), 5.0, 1e-9);
    EXPECT_NEAR(
        mpp::reeds_shepp_distance({0, 0, 0}, {R, R, M_PI / 2}, R), M_PI * R / 2,
        1e-9);
    EXPECT_NEAR(
        mpp::reeds_shepp_distance({1, 2, 0.3}, {1, 2, 0.3}, R), 0.0, 1e-9);
}

TEST(ReedsShepp, MetricProperties)
{
    std::mt19937                           rng(123);
    std::uniform_real_distribution<double> U(-5, 5);
    std::uniform_real_distribution<double> A(-M_PI, M_PI);
    const double                           R = 1.5;

    for (int i = 0; i < 2000; i++)
    {
        const mrpt::math::TPose2D a(U(rng), U(rng), A(rng));
        const mrpt::math::TPose2D b(U(rng), U(rng), A(rng));
        const mrpt::math::TPose2D c(U(rng), U(rng), A(rng));
        const double              ab = mpp::reeds_shepp_distance(a, b, R);
        const double              bc = mpp::reeds_shepp_distance(b, c, R);
        const double              ac = mpp::reeds_shepp_distance(a, c, R);

        EXPECT_NEAR(ab, mpp::reeds_shepp_distance(b, a, R), 1e-6);
        EXPECT_LE(ac, ab + bc + 1e-6);
        // Never shorter than the straight-line distance:
        EXPECT_GE(ab + 1e-9, std::hypot(b.x - a.x, b.y - a.y));
    }
}

TEST(ReedsShepp, LowerBoundOfPtgTrajectories)
{
    mrpt::config::CConfigFileMemory cfg(kCPtgForwardReverse);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    const double R = mpp::TPS_Astar::reeds_shepp_turning_radius(in.ptgs);
    EXPECT_NEAR(R, kTurningRadius, 1e-9);

    for (const auto& ptg : in.ptgs.ptgs)
    {
        ptg->initialize();
        for (size_t k = 0; k < ptg->getPathCount(); k++)
        {
            const auto nSteps = ptg->getPathStepCount(k);
            for (uint32_t n = 0; n < nSteps; n += 7)
            {
                const double arcLength =
                    ptg->getMaxLinVel() * n * ptg->getPathStepDuration();
                const double rs = mpp::reeds_shepp_distance(
                    {0, 0, 0}, ptg->getPathPose(k, n), R);
                EXPECT_LE(rs, arcLength + 1e-3) << "k=" << k << " n=" << n;
            }
        }
    }
}

TEST(ReedsShepp, HeuristicConsistentAndAdmissibleInPlan)
{
    mrpt::config::CConfigFileMemory cfg(kCPtgForwardReverse);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    in.stateStart.pose = {0, 0, 0};
    // Goal behind the start with the same heading: needs reverse motion.
    const mrpt::math::TPose2D goal{-2.0, 0.5, 0.0};
    in.stateGoal.state = goal;
    in.worldBboxMin    = {-6, -6, -M_PI};
    in.worldBboxMax    = {6, 6, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.10;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 15;
    planner.params_.ptg_sample_timestamps           = {0.25, 0.5, 1.0, 2.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 30.0;
    planner.params_.heuristic_epsilon               = 1.0;
    planner.params_.use_reeds_shepp_heuristic       = true;

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    mpp::SE2orR2_KinState g;
    g.state = goal;

    // Admissible at the start. The path ends anywhere inside the goal cell,
    // so compare with its cost plus the heuristic from its true endpoint.
    const auto [nodes, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    ASSERT_FALSE(edges.empty());
    const auto* last = edges.back();
    ASSERT_TRUE(last != nullptr);
    mpp::SE2_KinState endState;
    endState.pose = last->stateFrom.pose +
                    in.ptgs.ptgs.at(last->ptgIndex)
                        ->getPathPose(last->ptgPathIndex, last->ptgStepIndex);
    EXPECT_LE(
        planner.default_heuristic(in.stateStart, g),
        out.pathCost + planner.default_heuristic(endState, g) + 1e-6);
    // The Reeds-Shepp heuristic is the one in use:
    EXPECT_NEAR(
        planner.default_heuristic(in.stateStart, g),
        mpp::reeds_shepp_distance(in.stateStart.pose, goal, kTurningRadius) /
            1.0,
        1e-9);

    int edgeCount = 0;
    for (const auto& kv : out.motionTree.edges_to_children)
    {
        for (const auto& edgeEntry : kv.second)
        {
            const auto&  e    = edgeEntry.data;
            const double hU   = planner.default_heuristic(e.stateFrom, g);
            const double hV   = planner.default_heuristic(e.stateTo, g);
            const double cost = planner.cost_path_segment(e);
            EXPECT_LE(hU, cost + hV + 1e-3);
            edgeCount++;
        }
    }
    EXPECT_GT(edgeCount, 0);
}
