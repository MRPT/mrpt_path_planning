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

#include <chrono>

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

// PTG config with two HolonomicBlend entries — used for the multi-PTG test.
static const char* kTwoPtgCfg = R"cfg(
[SelfDriving]
min_obstacles_height  = 0.0
max_obstacles_height  = 2.0

PTG_COUNT = 2

PTG0_Type        = mpp::ptg::HolonomicBlend
PTG0_refDistance = 5.0
PTG0_num_paths   = 61
PTG0_T_ramp_max  = 1.0
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_expr_V      = V_MAX * trimmable_speed
PTG0_expr_W      = W_MAX * trimmable_speed * min(1.0, 0.1+abs(dir)/(10*3.14159265/180))
PTG0_expr_T_ramp = T_ramp_max

PTG1_Type        = mpp::ptg::HolonomicBlend
PTG1_refDistance = 5.0
PTG1_num_paths   = 31
PTG1_T_ramp_max  = 0.5
PTG1_v_max_mps   = 1.5
PTG1_w_max_dps   = 90.0
PTG1_expr_V      = V_MAX * trimmable_speed
PTG1_expr_W      = W_MAX * trimmable_speed * min(1.0, 0.1+abs(dir)/(10*3.14159265/180))
PTG1_expr_T_ramp = T_ramp_max

RobotModel_circular_shape_radius = 0.15
)cfg";

// PTG config with v_max = 2.0 m/s — used for the normalisation regression test.
static const char* kHolonomicPtgCfg2mps = R"cfg(
[SelfDriving]
min_obstacles_height  = 0.0
max_obstacles_height  = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::HolonomicBlend
PTG0_refDistance = 5.0
PTG0_num_paths   = 61
PTG0_T_ramp_max  = 1.0
PTG0_v_max_mps   = 2.0
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
    const mrpt::maps::CSimplePointsMap::Ptr& obsPts = nullptr,
    const char*                              ptgCfg = kHolonomicPtgCfg)
{
    // Load PTGs
    mrpt::config::CConfigFileMemory cfg(ptgCfg);
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

TEST(AstarHolonomic, NonZeroGoalSpeedPreserved)
{
    // When stateGoal.vel carries a non-zero exit velocity, the final planned
    // edge must reflect a non-zero ptgFinalGoalRelSpeed so the robot does not
    // decelerate to a full stop when chaining waypoints.
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/3.0, /*gy=*/0.0);

    in.stateGoal.vel = mrpt::math::TTwist2D{0.5, 0.0, 0.0};  // 0.5 m/s fwd

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    ASSERT_TRUE(out.goalNodeId.has_value());

    const auto& last_edge =
        out.motionTree.edge_to_parent(out.goalNodeId.value());
    EXPECT_GT(last_edge.ptgFinalGoalRelSpeed, 0.01)
        << "Final edge must carry non-zero goal speed when stateGoal.vel is "
           "set";
}

TEST(AstarHolonomic, ZeroGoalSpeedDefault)
{
    // When stateGoal.vel is unset (zero), the planner must preserve the
    // original stop-at-goal behaviour.
    auto planner = buildPlanner();
    auto in      = buildInput(/*gx=*/2.0, /*gy=*/0.0);
    // stateGoal.vel intentionally left at default TTwist2D{0,0,0}.

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    ASSERT_TRUE(out.goalNodeId.has_value());

    const auto& last_edge =
        out.motionTree.edge_to_parent(out.goalNodeId.value());
    EXPECT_NEAR(last_edge.ptgFinalGoalRelSpeed, 0.0, 0.05)
        << "Robot should stop at goal when stateGoal.vel is unset";
}

TEST(AstarHolonomic, GoalSpeedNormalisedAgainstPtgVmax)
{
    // Use a PTG with v_max = 2.0 m/s.  Request an exit speed of 1.0 m/s
    // (= 50 % of v_max).  The stored normalised speed must be ≈ 0.5, NOT 1.0.
    // This guards against the regression where hypot(vx,vy) was used directly
    // without dividing by the PTG's own maximum linear velocity.
    auto planner = buildPlanner();
    auto in      = buildInput(
             /*gx=*/3.0, /*gy=*/0.0,
        /*goalIsSE2=*/false, /*goalPhi_deg=*/0.0,
        /*obsPts=*/nullptr, kHolonomicPtgCfg2mps);

    // Request 1.0 m/s exit speed; PTG v_max = 2.0 m/s → expected relSpeed ≈ 0.5
    in.stateGoal.vel = mrpt::math::TTwist2D{1.0, 0.0, 0.0};

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success);
    ASSERT_TRUE(out.goalNodeId.has_value());

    const auto& last_edge =
        out.motionTree.edge_to_parent(out.goalNodeId.value());

    // Must be non-zero (goal speed wired in).
    EXPECT_GT(last_edge.ptgFinalGoalRelSpeed, 0.01)
        << "Goal speed must be propagated when stateGoal.vel is set";

    // Must be well below 1.0 — if normalisation was missing the value would
    // have been clamped to 1.0 (full speed), not ≈ 0.5.
    EXPECT_LT(last_edge.ptgFinalGoalRelSpeed, 0.9)
        << "Normalised goal speed must account for PTG v_max (2 m/s), "
           "so 1 m/s requested should give ~0.5, not 1.0";
}

TEST(AstarHolonomic, TrimSpeedPreservedInBestPath)
{
    // Use max_ptg_speeds_to_explore >= 2 so the planner evaluates a reduced
    // speed level. If a lower-speed path wins, ptgTrimmableSpeed must reflect
    // that speed rather than the default 1.0.
    auto planner                              = buildPlanner();
    planner.params_.max_ptg_speeds_to_explore = 3;

    auto in  = buildInput(/*gx=*/3.0, /*gy=*/0.0);
    auto out = planner.plan(in);

    ASSERT_TRUE(out.success);

    // Walk all edges in the motion tree; at least one must have a trimmed
    // speed < 1.0 (the planner explored non-maximum speeds).
    bool found_trimmed = false;
    for (const auto& kv : out.motionTree.edges_to_children)
    {
        for (const auto& edge : kv.second)
        {
            if (edge.data.ptgTrimmableSpeed < 0.999) found_trimmed = true;
        }
    }

    EXPECT_TRUE(found_trimmed)
        << "No edge has ptgTrimmableSpeed < 1.0; speed is being lost";
}

TEST(AstarHolonomic, UnreachableGoal)
{
    // Dense ring of obstacles at 0.2 m from the start (0,0).
    // Every trajectory leaving the start immediately hits an obstacle —
    // the start cell is visited only once so the obstacle cloud is always
    // computed fresh (no cache-aliasing risk).
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();
    for (double a = 0; a < 2 * M_PI; a += 0.05)
        obsPts->insertPoint(0.2 * std::cos(a), 0.2 * std::sin(a), 0.0);

    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 5.0;

    auto       in  = buildInput(2.0, 0.0, false, 0.0, obsPts);
    const auto out = planner.plan(in);

    EXPECT_FALSE(out.success)
        << "Robot completely enclosed by obstacles must fail to plan";
}

TEST(AstarHolonomic, TimeoutRespected)
{
    // maximumComputationTime = 0.0 guarantees the planner exits after its
    // first iteration timeout check regardless of the problem.
    auto planner                           = buildPlanner();
    planner.params_.maximumComputationTime = 0.0;

    auto in = buildInput(3.0, 0.0);

    const auto t0  = std::chrono::steady_clock::now();
    const auto out = planner.plan(in);
    const auto t1  = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(t1 - t0).count();

    EXPECT_FALSE(out.success);
    EXPECT_LT(elapsed, 1.0)
        << "Planner exceeded 1 s wall-clock; elapsed=" << elapsed << " s";
}

TEST(AstarHolonomic, AnalyticExpansionEarlyTermination)
{
    // Analytic expansion (default on) terminates as soon as a collision-free
    // connection into the goal cell is found. It must still reach the goal and
    // must not expand MORE nodes than the strictly-optimal (analytic off) mode.
    auto plannerOff                           = buildPlanner();
    plannerOff.params_.use_analytic_expansion = false;
    auto       inOff  = buildInput(/*gx=*/3.0, /*gy=*/0.0);
    const auto outOff = plannerOff.plan(inOff);
    ASSERT_TRUE(outOff.success);

    auto plannerOn                           = buildPlanner();
    plannerOn.params_.use_analytic_expansion = true;
    auto       inOn  = buildInput(/*gx=*/3.0, /*gy=*/0.0);
    const auto outOn = plannerOn.plan(inOn);
    ASSERT_TRUE(outOn.success)
        << "Analytic expansion must still reach the goal";

    EXPECT_LE(outOn.motionTree.nodes().size(), outOff.motionTree.nodes().size())
        << "Analytic expansion should not expand more nodes than optimal mode "
           "(on="
        << outOn.motionTree.nodes().size()
        << ", off=" << outOff.motionTree.nodes().size() << ")";
}

TEST(AstarHolonomic, MultiPTG)
{
    // Plan with two PTG entries. Planner must succeed and every edge in the
    // motion tree must reference a valid PTG index (0 or 1).
    auto planner = buildPlanner();
    auto in =
        buildInput(/*gx=*/2.0, /*gy=*/1.0, false, 0.0, nullptr, kTwoPtgCfg);

    const auto out = planner.plan(in);
    ASSERT_TRUE(out.success) << "Planner must find a path with 2 PTGs";

    for (const auto& kv : out.motionTree.edges_to_children)
    {
        for (const auto& edgeEntry : kv.second)
        {
            EXPECT_GE(edgeEntry.data.ptgIndex, 0)
                << "Edge has negative ptgIndex";
            EXPECT_LT(edgeEntry.data.ptgIndex, 2)
                << "Edge ptgIndex out of range for a 2-PTG plan";
        }
    }
}
