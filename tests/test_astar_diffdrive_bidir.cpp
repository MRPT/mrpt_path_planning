/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * End-to-end integration tests for the bidirectional planner TPS_Astar_Bidir,
 * mirroring the forward-only tests in test_astar_diffdrive.cpp. The same coarse
 * C-PTG configs are used so collision grids build fast.
 *
 * Verified behaviors:
 *  - Reaches a goal straight ahead (forward + backward frontiers meet).
 *  - Matches forward-only path cost within tolerance on an easy world.
 *  - Reaches an SE(2) goal pose with the requested heading.
 *  - Reaches an R(2) point goal (multi-root backward seeding).
 *  - Uses the reverse PTG to reach a goal behind in a narrow corridor.
 *  - Fails (no tunneling / no discontinuous stitch) through a solid wall.
 *  - The stitched output path is continuous after refine_trajectory().
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/TPS_Astar_Bidir.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/wrap2pi.h>

#include <algorithm>
#include <vector>

// ---------------------------------------------------------------------------
// PTG configs (same as test_astar_diffdrive.cpp).
// ---------------------------------------------------------------------------
static const char* kCPtgForward = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0
PTG_COUNT = 1
PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 4.0
PTG0_num_paths   = 41
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0
RobotModel_shape2D_xs = -0.20 0.30 0.30 -0.20
RobotModel_shape2D_ys = -0.18 -0.18 0.18 0.18
)cfg";

static const char* kCPtgForwardReverse = R"cfg(
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

static constexpr int kReversePtgIndex = 1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static mpp::PlannerInput buildInput(
    const mpp::PoseOrPoint& goal, const char* ptgCfg,
    const mrpt::math::TPose2D& bboxMin, const mrpt::math::TPose2D& bboxMax,
    const mrpt::maps::CSimplePointsMap::Ptr& obsPts = nullptr)
{
    mrpt::config::CConfigFileMemory cfg(ptgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = goal;

    in.worldBboxMin = bboxMin;
    in.worldBboxMax = bboxMax;

    if (obsPts)
        in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obsPts));

    return in;
}

static mrpt::maps::CSimplePointsMap::Ptr buildCorridorWalls(
    double x0, double x1, double halfWidth)
{
    auto       pts = mrpt::maps::CSimplePointsMap::Create();
    const auto hw  = static_cast<float>(halfWidth);
    for (double x = x0; x <= x1; x += 0.1)
    {
        const auto fx = static_cast<float>(x);
        pts->insertPoint(fx, +hw, 0.0f);
        pts->insertPoint(fx, -hw, 0.0f);
    }
    return pts;
}

static mrpt::maps::CSimplePointsMap::Ptr buildSealedBoxWithDivider(
    double x0, double x1, double y0, double y1, double xdiv)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (double x = x0; x <= x1; x += 0.05)
    {
        pts->insertPoint(static_cast<float>(x), static_cast<float>(y0), 0.0f);
        pts->insertPoint(static_cast<float>(x), static_cast<float>(y1), 0.0f);
    }
    for (double y = y0; y <= y1; y += 0.05)
    {
        pts->insertPoint(static_cast<float>(x0), static_cast<float>(y), 0.0f);
        pts->insertPoint(static_cast<float>(x1), static_cast<float>(y), 0.0f);
        pts->insertPoint(static_cast<float>(xdiv), static_cast<float>(y), 0.0f);
    }
    return pts;
}

template <class PLANNER>
static void configurePlanner(PLANNER& planner)
{
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 20;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.5, 3.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 20.0;
    planner.params_.SE2_metricAngleWeight           = 0.1;
    planner.params_.heuristic_heading_weight        = 0.1;
}

static std::vector<int> solutionPtgIndices(const mpp::PlannerOutput& out)
{
    std::vector<int> indices;
    if (!out.success || !out.goalNodeId.has_value()) return indices;
    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());
    for (const auto* edge : edges)
        if (edge != nullptr) indices.push_back(edge->ptgIndex);
    return indices;
}

// Walk the solution edges and return the largest pose discontinuity between
// consecutive edges (xy and yaw), plus total length.
struct Continuity
{
    double maxXYGap  = 0;
    double maxYawGap = 0;
    double length    = 0;
};

static Continuity measureContinuity(const mpp::PlannerOutput& out)
{
    Continuity c;
    if (!out.success || !out.goalNodeId.has_value()) return c;
    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());

    const mpp::MoveEdgeSE2_TPS* prev = nullptr;
    for (const auto* e : edges)
    {
        if (e == nullptr) continue;
        if (prev != nullptr)
        {
            c.maxXYGap = std::max(
                c.maxXYGap, std::hypot(
                                e->stateFrom.pose.x - prev->stateTo.pose.x,
                                e->stateFrom.pose.y - prev->stateTo.pose.y));
            c.maxYawGap = std::max(
                c.maxYawGap, std::abs(mrpt::math::wrapToPi(
                                 e->stateFrom.pose.phi - prev->stateTo.pose.phi)));
        }
        c.length += std::hypot(
            e->stateTo.pose.x - e->stateFrom.pose.x,
            e->stateTo.pose.y - e->stateFrom.pose.y);
        prev = e;
    }
    return c;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(AstarDiffDriveBidir, BidirReachesGoalAhead)
{
    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    auto in = buildInput(
        mrpt::math::TPoint2D{3.0, 0.0}, kCPtgForward, {-2, -3, -M_PI},
        {5, 3, M_PI});

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success)
        << "Bidirectional C-PTG must reach a goal straight ahead at (3,0)";
}

TEST(AstarDiffDriveBidir, BidirMatchesForwardOnEasyWorld)
{
    auto in = buildInput(
        mrpt::math::TPoint2D{2.5, 1.5}, kCPtgForward, {-2, -2, -M_PI},
        {5, 4, M_PI});

    mpp::TPS_Astar fwd;
    configurePlanner(fwd);
    const auto outF = fwd.plan(in);
    ASSERT_TRUE(outF.success);

    mpp::TPS_Astar_Bidir bidir;
    configurePlanner(bidir);
    const auto outB = bidir.plan(in);
    ASSERT_TRUE(outB.success);

    // Quality-not-worse gate (unit form): bidirectional cost within tolerance.
    EXPECT_LT(outB.pathCost, 1.30 * outF.pathCost)
        << "bidir cost " << outB.pathCost << " vs forward " << outF.pathCost;
}

TEST(AstarDiffDriveBidir, BidirSE2GoalReachesExactHeading)
{
    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    auto in = buildInput(
        mrpt::math::TPose2D{2.5, 1.5, 90.0 * M_PI / 180.0}, kCPtgForward,
        {-2, -2, -M_PI}, {5, 4, M_PI});

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success) << "SE(2) goal must be reachable";

    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());
    const double finalPhi = nodes.back().pose.phi;
    const double err =
        std::abs(mrpt::math::wrapToPi(finalPhi - 90.0 * M_PI / 180.0));
    EXPECT_LT(err, 2.0 * planner.params_.grid_resolution_yaw)
        << "final heading " << finalPhi * 180.0 / M_PI << " deg off target";
}

TEST(AstarDiffDriveBidir, BidirR2GoalAnyHeading)
{
    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    auto in = buildInput(
        mrpt::math::TPoint2D{2.5, 1.5}, kCPtgForward, {-2, -2, -M_PI},
        {5, 4, M_PI});

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success) << "R(2) point goal must be reachable";

    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());
    const auto& fp = nodes.back().pose;
    EXPECT_LT(std::hypot(fp.x - 2.5, fp.y - 1.5), 0.5)
        << "did not arrive at the goal xy";
}

TEST(AstarDiffDriveBidir, BidirReverseReachesGoalBehind)
{
    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    planner.params_.maximumComputationTime = 20.0;

    auto walls = buildCorridorWalls(-3.0, 3.0, 0.7);
    auto in    = buildInput(
        mrpt::math::TPoint2D{-1.5, 0.0}, kCPtgForwardReverse, {-3, -1.0, -M_PI},
        {3, 1.0, M_PI}, walls);

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success)
        << "Forward+reverse C-PTG set must reach a goal straight behind";

    const auto indices = solutionPtgIndices(out);
    ASSERT_FALSE(indices.empty());
    const bool usedReverse =
        std::find(indices.begin(), indices.end(), kReversePtgIndex) !=
        indices.end();
    EXPECT_TRUE(usedReverse)
        << "Reaching a goal behind in a narrow corridor must use the reverse "
           "PTG";
}

TEST(AstarDiffDriveBidir, BidirSolidWallBlocksGoal)
{
    auto box = buildSealedBoxWithDivider(0.0, 2.0, -1.0, 1.0, 1.0);

    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    planner.params_.maximumComputationTime = 10.0;

    auto in = buildInput(
        mrpt::math::TPoint2D{1.5, 0.0}, kCPtgForward, {0.0, -1.0, -M_PI},
        {2.0, 1.0, M_PI}, box);
    in.stateStart.pose = {0.5, 0.0, 0.0};

    const auto out = planner.plan(in);
    EXPECT_FALSE(out.success)
        << "Bidirectional planner must NOT stitch a path through a solid wall";
}

TEST(AstarDiffDriveBidir, BidirStitchedPathIsContinuous)
{
    mpp::TPS_Astar_Bidir planner;
    configurePlanner(planner);
    auto in = buildInput(
        mrpt::math::TPoint2D{3.0, 0.0}, kCPtgForward, {-2, -3, -M_PI},
        {5, 3, M_PI});

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);

    const auto c = measureContinuity(out);
    // After refine_trajectory(), consecutive edges must share endpoints to
    // tight tolerance (no visible discontinuity at the meeting point).
    EXPECT_LT(c.maxXYGap, 1e-3)
        << "xy discontinuity " << c.maxXYGap << " m at a stitched join";
    EXPECT_LT(c.maxYawGap, 1e-3)
        << "yaw discontinuity " << c.maxYawGap << " rad at a stitched join";
}
