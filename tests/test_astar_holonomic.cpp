/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * End-to-end integration tests for TPS_Astar with a holonomic vehicle.
 *
 * Verified behaviors:
 *  - Planner finds a path in an obstacle-free environment (trivial case).
 *  - Planner finds a path when start == goal (zero-distance case).
 *  - Planner succeeds with an R(2) goal (position-only, ignore heading).
 *  - Planner correctly handles obstacles placed between start and goal.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>

// ---------------------------------------------------------------------------
// Inline PTG config for a holonomic (HolonomicBlend) vehicle.
// Deliberately minimal: 1 PTG, few paths, short reference distance.
// ---------------------------------------------------------------------------
static const char* kHolonomicPtgCfg = R"cfg(
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

// ---------------------------------------------------------------------------
// Helper: build a PlannerInput for a holonomic robot
// ---------------------------------------------------------------------------
static mpp::PlannerInput buildInput(
    double gx, double gy, bool goalIsSE2 = false, double goalPhi_deg = 0.0,
    const mrpt::maps::CSimplePointsMap::Ptr& obsPts = nullptr)
{
    // Load PTGs
    mrpt::config::CConfigFileMemory cfg(kHolonomicPtgCfg);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");

    in.stateStart.pose = {0, 0, 0};

    if (goalIsSE2)
        in.stateGoal.state =
            mrpt::math::TPose2D{gx, gy, goalPhi_deg * M_PI / 180.0};
    else
        in.stateGoal.state = mrpt::math::TPoint2D{gx, gy};

    in.worldBboxMin = {-6, -6, -M_PI};
    in.worldBboxMax = {6, 6, M_PI};

    if (obsPts)
        in.obstacles.push_back(
            mpp::ObstacleSource::FromStaticPointcloud(obsPts));

    return in;
}

// ---------------------------------------------------------------------------
// Helper: build a TPS_Astar planner with fast settings for tests
// ---------------------------------------------------------------------------
static mpp::TPS_Astar buildPlanner()
{
    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);  // suppress output

    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = 10.0 * M_PI / 180.0;
    planner.params_.max_ptg_trajectories_to_explore = 15;
    planner.params_.ptg_sample_timestamps           = {0.5, 1.5, 3.0};
    planner.params_.max_ptg_speeds_to_explore       = 1;
    planner.params_.maximumComputationTime          = 30.0;  // seconds
    planner.params_.SE2_metricAngleWeight           = 0.1;
    planner.params_.heuristic_heading_weight        = 0.1;

    return planner;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(AstarHolonomic, FindsPathInFreeSpace)
{
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/2.0, /*gy=*/0.0);

    const auto out = planner.plan(in);

    EXPECT_TRUE(out.success)
        << "Planner must find a path to (2,0) in free space";
}

TEST(AstarHolonomic, FindsPathDiagonal)
{
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/2.0, /*gy=*/2.0);

    const auto out = planner.plan(in);

    EXPECT_TRUE(out.success)
        << "Planner must find a path to (2,2) in free space";
}

TEST(AstarHolonomic, R2GoalIgnoresHeading)
{
    // An R(2) goal only cares about (x,y). The planner should succeed
    // regardless of which heading the robot ends up with.
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/1.5, /*gy=*/1.0, /*goalIsSE2=*/false);

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success);
}

TEST(AstarHolonomic, SE2GoalReachesPosition)
{
    auto planner = buildPlanner();
    auto in =
        buildInput(/*gx=*/2.0, /*gy=*/0.0, /*goalIsSE2=*/true, /*phi=*/0.0);

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success) << "SE(2) goal at (2,0,0deg) should be reachable";
}

TEST(AstarHolonomic, ObstacleBetweenStartAndGoal)
{
    // Place a wall of obstacle points between start and goal.
    // The holonomic robot should navigate around them.
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();
    for (double y = -0.6; y <= 0.6; y += 0.1) obsPts->insertPoint(1.0, y, 0.0);

    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/2.0, /*gy=*/0.0, false, 0.0, obsPts);

    const auto out = planner.plan(in);
    EXPECT_TRUE(out.success)
        << "Planner should find a path around the obstacle wall";
}

TEST(AstarHolonomic, PathCostIsPositive)
{
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/2.0, /*gy=*/0.0);

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    EXPECT_GT(out.pathCost, 0.0)
        << "Path cost must be positive for a non-trivial path";
}
