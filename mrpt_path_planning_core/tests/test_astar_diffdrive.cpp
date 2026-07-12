/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * End-to-end integration tests for TPS_Astar with a non-holonomic
 * differential-drive / Ackermann vehicle using the circular-arc "C" PTG
 * (mpp::ptg::DiffDrive_C).
 *
 * These complement test_astar_holonomic.cpp (which covers HolonomicBlend) and
 * exercise the PTG family the paper keeps: the "C" PTG in both its forward
 * (K=+1) and reverse (K=-1) variants.
 *
 * Verified behaviors:
 *  - Forward C-PTG reaches a goal straight ahead.
 *  - Forward C-PTG curves to an offset goal, using only forward motion.
 *  - A forward-only PTG set cannot reach a goal straight behind the robot
 *    inside a narrow corridor (no room to turn around).
 *  - Adding a reverse (K=-1) C-PTG makes that same goal reachable, and the
 *    solution path actually uses the reverse PTG.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <algorithm>
#include <vector>

// ---------------------------------------------------------------------------
// PTG configs. Deliberately coarse (few paths, short reference distance, coarse
// collision-grid resolution) so the collision grids build fast in unit tests.
// Robot is a small rectangular polygon (non-circular, to exercise the any-shape
// collision path). v_max=1 m/s, w_max=60 deg/s give a min turning radius of
// about 0.95 m, which matters for the corridor / reverse tests below.
// ---------------------------------------------------------------------------

// Single forward C-PTG (K=+1).
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

// Forward (K=+1) plus reverse (K=-1) C-PTGs.
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

// Index of the reverse PTG in kCPtgForwardReverse.
static constexpr int kReversePtgIndex = 1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static mpp::PlannerInput buildInput(
    double gx, double gy, const char* ptgCfg,
    const mrpt::math::TPose2D& bboxMin, const mrpt::math::TPose2D& bboxMax,
    const mrpt::maps::CSimplePointsMap::Ptr& obsPts = nullptr)
{
    mrpt::config::CConfigFileMemory cfg(ptgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    in.stateStart.pose = {0, 0, 0};
    in.stateGoal.state = mrpt::math::TPoint2D{gx, gy};  // R(2) goal

    in.worldBboxMin = bboxMin;
    in.worldBboxMax = bboxMax;

    if (obsPts)
    {
        in.obstacles.push_back(
            mpp::ObstacleSource::FromStaticPointcloud(obsPts));
    }

    return in;
}

// Two solid walls at y = +/- halfWidth spanning x in [x0,x1], so the free
// space between them is a corridor too narrow for the vehicle to turn around
// (min turning radius ~0.95 m needs ~1.9 m of width). Reversing straight is the
// only way to reach a goal behind the robot.
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

// A sealed rectangular box [x0,x1] x [y0,y1] (dense border points) with a solid
// vertical divider at x=xdiv spanning the full height. The box prevents the
// swept arcs from bulging "around" the obstacles, so a goal on the far side of
// the divider is genuinely unreachable. Regression guard for the obstacle-cache
// collision-soundness bug (the planner used to tunnel straight through).
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

static mpp::TPS_Astar buildPlanner()
{
    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);

    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 20;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.5, 3.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 20.0;
    planner.params_.SE2_metricAngleWeight           = 0.1;
    planner.params_.heuristic_heading_weight        = 0.1;

    return planner;
}

// Returns the PTG indices of the edges along the solution path
// (root -> goal). Empty if no solution.
static std::vector<int> solutionPtgIndices(const mpp::PlannerOutput& out)
{
    std::vector<int> indices;
    if (!out.success || !out.goalNodeId.has_value()) { return indices; }

    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());
    for (const auto* edge : edges)
    {
        if (edge != nullptr) { indices.push_back(edge->ptgIndex); }
    }
    return indices;
}

// Geometric quality of a solution path, accumulated over the interpolated
// poses of all solution edges (poses are relative to each edge's stateFrom).
struct PathQuality
{
    double length      = 0;  //!< total travelled XY arc length [m]
    double totalTurn   = 0;  //!< sum of |heading increments| [rad]
    size_t numSamples  = 0;
};

static PathQuality measurePath(const mpp::PlannerOutput& out)
{
    PathQuality q;
    if (!out.success || !out.goalNodeId.has_value()) { return q; }

    const auto [nodes, edges] =
        out.motionTree.backtrack_path(out.goalNodeId.value());

    mrpt::math::TPose2D prev;
    bool                first = true;
    for (const auto* e : edges)
    {
        if (e == nullptr) { continue; }
        for (const auto& [t, rel] : e->interpolatedPath)
        {
            const auto abs = e->stateFrom.pose + rel;
            if (!first)
            {
                q.length += std::hypot(abs.x - prev.x, abs.y - prev.y);
                q.totalTurn +=
                    std::abs(mrpt::math::wrapToPi(abs.phi - prev.phi));
            }
            prev = abs;
            first = false;
            q.numSamples++;
        }
    }
    return q;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Regression test for the suboptimal "loop near the goal" artifact visible in
// the paper's fig:cases (left panel): a robot crossing a doorway arrived at the
// goal position with its heading ~12 deg off and then performed a full ~360 deg
// in-place loop to re-enter the exact goal SE(2) lattice cell, instead of a
// gentle approach. Root cause: the analytic goal expansion used to terminate on
// the *first* collision-free edge landing in the goal cell. With a tight turning
// radius, a node already at the goal xy but a couple of yaw cells off can only
// reach the exact goal cell through a near-full-circle PTG arc, so that greedy
// first edge was a loop. The fix records the goal-landing edge as a candidate
// and commits only when it is provably (eps-)optimal.
//
// This reproduces the geometry in open space (no obstacles) with the same
// PTG/turning-radius regime, and asserts the easy path is NOT over-complicated.
TEST(AstarDiffDrive, EasyGoalHasNoExcessiveLoop)
{
    // Forward+reverse C-PTG, v_max=0.4 m/s, w_max=90 deg/s -> min turning
    // radius ~0.25 m, the tight-radius regime that made the loop possible.
    static const char* kTightTurn = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0
PTG_COUNT = 2
PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.05
PTG0_refDistance = 3.0
PTG0_num_paths   = 121
PTG0_v_max_mps   = 0.4
PTG0_w_max_dps   = 90.0
PTG0_K           = +1.0
PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.05
PTG1_refDistance = 3.0
PTG1_num_paths   = 121
PTG1_v_max_mps   = 0.4
PTG1_w_max_dps   = 90.0
PTG1_K           = -1.0
RobotModel_shape2D_xs = -0.30 0.50 0.65 0.65 0.50 -0.30
RobotModel_shape2D_ys = 0.275 0.275 0.20 -0.20 -0.275 -0.275
)cfg";

    mrpt::config::CConfigFileMemory cfg(kTightTurn);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    // Mirrors fig:cases left panel: start at 45 deg, SE(2) goal ~3.4 m away at
    // 90 deg. A gentle curve aligns naturally; no loop is needed.
    in.stateStart.pose = {0, 0, 45.0 * M_PI / 180.0};
    in.stateGoal.state = mrpt::math::TPose2D{0.6, 3.35, 90.0 * M_PI / 180.0};
    in.worldBboxMin    = {-3, -1, -M_PI};
    in.worldBboxMax    = {3, 5, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.10;
    planner.params_.grid_resolution_yaw             = 7.5 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 30;
    planner.params_.ptg_sample_timestamps           = {1.0, 3.0, 5.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 30.0;
    planner.params_.SE2_metricAngleWeight           = 1.0;
    planner.params_.heuristic_heading_weight        = 0.1;

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success) << "Easy open-space SE(2) goal must be solvable";

    const auto q        = measurePath(out);
    const double straight = std::hypot(0.6, 3.35);

    // The net required heading change is only 45 deg. Anything close to a full
    // revolution (the old artifact accumulated ~406 deg) is a spurious loop.
    EXPECT_LT(q.totalTurn, M_PI)
        << "Path winds " << q.totalTurn * 180.0 / M_PI
        << " deg (>180): a spurious near-full-circle loop near the goal";

    // An easy gentle curve should be only marginally longer than the straight
    // line. The looping solution was ~1.6x; a clean approach is ~1.0-1.1x.
    EXPECT_LT(q.length, 1.3 * straight)
        << "Path length " << q.length << " m is >1.3x the straight-line "
        << straight << " m: over-complicated solution";
}

TEST(AstarDiffDrive, ForwardReachesGoalAhead)
{
    auto planner = buildPlanner();
    auto in      = buildInput(
             /*gx=*/3.0, /*gy=*/0.0, kCPtgForward,
        /*bboxMin=*/{-2, -3, -M_PI}, /*bboxMax=*/{5, 3, M_PI});

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success)
        << "Forward C-PTG must reach a goal straight ahead at (3,0)";
}

TEST(AstarDiffDrive, ForwardCurvesToOffsetGoal)
{
    auto planner = buildPlanner();
    auto in      = buildInput(
             /*gx=*/2.5, /*gy=*/1.5, kCPtgForward,
        /*bboxMin=*/{-2, -2, -M_PI}, /*bboxMax=*/{5, 4, M_PI});

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success)
        << "Forward C-PTG must curve to reach an offset goal at (2.5,1.5)";
    EXPECT_GT(out.pathCost, 0.0);

    // Every edge must use the single (forward) PTG, index 0.
    const auto indices = solutionPtgIndices(out);
    ASSERT_FALSE(indices.empty());
    for (const int idx : indices)
    {
        EXPECT_EQ(idx, 0) << "Forward-only plan must only use PTG index 0";
    }
}

TEST(AstarDiffDrive, ForwardOnlyCannotReachBehindInCorridor)
{
    // Goal directly behind the start, inside a walled corridor of half-width
    // 0.7 m (total 1.4 m < ~1.9 m needed to U-turn at min radius). A forward
    // arc can never decrease x below the start without turning around, and the
    // walls block any turn-around maneuver. So forward-only planning must fail.
    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 10.0;

    auto walls = buildCorridorWalls(/*x0=*/-3.0, /*x1=*/3.0, /*halfWidth=*/0.7);
    auto in    = buildInput(
           /*gx=*/-1.5, /*gy=*/0.0, kCPtgForward,
        /*bboxMin=*/{-3, -1.0, -M_PI}, /*bboxMax=*/{3, 1.0, M_PI}, walls);

    const auto out = planner.plan(in);
    EXPECT_FALSE(out.success)
        << "Forward-only C-PTG must fail to reach a goal behind the robot in a "
           "corridor too narrow to turn around";
}

TEST(AstarDiffDrive, SolidWallBlocksGoal)
{
    // A sealed box split by a solid divider at x=1.0; start on the left, goal
    // on the right. The goal is genuinely unreachable, so the planner MUST
    // fail. Guards against the obstacle-cache bug where nodes sharing an
    // xy-cell but with different headings reused mis-rotated obstacles, letting
    // the planner tunnel straight through solid walls.
    auto box = buildSealedBoxWithDivider(
        /*x0=*/0.0, /*x1=*/2.0, /*y0=*/-1.0, /*y1=*/1.0, /*xdiv=*/1.0);

    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 10.0;

    auto in = buildInput(
        /*gx=*/1.5, /*gy=*/0.0, kCPtgForward,
        /*bboxMin=*/{0.0, -1.0, -M_PI}, /*bboxMax=*/{2.0, 1.0, M_PI}, box);
    in.stateStart.pose = {0.5, 0.0, 0.0};

    const auto out = planner.plan(in);
    EXPECT_FALSE(out.success)
        << "Planner must NOT find a path through a solid wall (collision "
           "soundness)";
}

TEST(AstarDiffDrive, ReverseReachesGoalBehind)
{
    // Same walled corridor and goal as above, but now with a reverse (K=-1)
    // C-PTG added. The robot can simply back straight up to the goal.
    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 10.0;

    auto walls = buildCorridorWalls(/*x0=*/-3.0, /*x1=*/3.0, /*halfWidth=*/0.7);
    auto in    = buildInput(
           /*gx=*/-1.5, /*gy=*/0.0, kCPtgForwardReverse,
        /*bboxMin=*/{-3, -1.0, -M_PI}, /*bboxMax=*/{3, 1.0, M_PI}, walls);

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success)
        << "Forward+reverse C-PTG set must reach a goal straight behind";

    // The solution must actually use the reverse PTG at least once.
    const auto indices = solutionPtgIndices(out);
    ASSERT_FALSE(indices.empty());
    const bool usedReverse =
        std::find(indices.begin(), indices.end(), kReversePtgIndex) !=
        indices.end();
    EXPECT_TRUE(usedReverse)
        << "Reaching a goal behind in a narrow corridor must use the reverse "
           "PTG (index "
        << kReversePtgIndex << ")";
}
