/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for TPS_Astar heuristic functions.
 *
 * Verified behaviors:
 *  - Heuristic is non-negative (admissibility requirement).
 *  - Heuristic is zero at the goal (h(goal, goal) == 0).
 *  - SE(2) heuristic with angle weight = 0 matches R(2) heuristic.
 *  - R(2) heuristic depends only on position, not on heading.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mrpt/core/bits_math.h>  // _deg

using namespace mrpt::literals;  // for _deg

static mpp::SE2_KinState makeState(double x, double y, double phi_deg = 0.0)
{
    mpp::SE2_KinState s;
    s.pose = {x, y, phi_deg * M_PI / 180.0};
    return s;
}

static mpp::SE2orR2_KinState makeGoalSE2(
    double x, double y, double phi_deg = 0.0)
{
    mpp::SE2orR2_KinState g;
    g.state = mrpt::math::TPose2D{x, y, phi_deg * M_PI / 180.0};
    return g;
}

static mpp::SE2orR2_KinState makeGoalR2(double x, double y)
{
    mpp::SE2orR2_KinState g;
    g.state = mrpt::math::TPoint2D{x, y};
    return g;
}

// ---- Tests ------------------------------------------------------------------

TEST(Heuristic, NonNegative)
{
    mpp::TPS_Astar planner;
    planner.params_.SE2_metricAngleWeight    = 1.0;
    planner.params_.heuristic_heading_weight = 0.1;

    const auto from = makeState(0, 0, 0);
    EXPECT_GE(planner.default_heuristic(from, makeGoalSE2(3, 4, 45)), 0.0);
    EXPECT_GE(planner.default_heuristic(from, makeGoalR2(3, 4)), 0.0);
    EXPECT_GE(planner.default_heuristic(from, makeGoalSE2(-1, -1, 180)), 0.0);
}

TEST(Heuristic, ZeroAtGoalSE2)
{
    mpp::TPS_Astar planner;

    const auto state   = makeState(1.5, -2.0, 30.0);
    const auto goalSE2 = makeGoalSE2(1.5, -2.0, 30.0);

    EXPECT_NEAR(planner.default_heuristic(state, goalSE2), 0.0, 1e-9);
}

TEST(Heuristic, ZeroAtGoalR2)
{
    mpp::TPS_Astar planner;

    const auto state  = makeState(1.5, -2.0, 45.0);
    const auto goalR2 = makeGoalR2(1.5, -2.0);

    // For R(2) goal, heading is irrelevant — heuristic should be 0 when
    // position matches regardless of phi.
    EXPECT_NEAR(planner.default_heuristic(state, goalR2), 0.0, 1e-9);
}

TEST(Heuristic, SE2WithZeroAngleWeightMatchesR2)
{
    // When SE2_metricAngleWeight = 0 and heuristic_heading_weight = 0,
    // the SE(2) heuristic should collapse to pure R(2) Euclidean distance.
    mpp::TPS_Astar planner;
    planner.params_.SE2_metricAngleWeight    = 0.0;
    planner.params_.heuristic_heading_weight = 0.0;

    const auto from    = makeState(0, 0, 30.0);
    const auto goalSE2 = makeGoalSE2(3.0, 4.0, 90.0);
    const auto goalR2  = makeGoalR2(3.0, 4.0);

    const double hSE2 = planner.default_heuristic(from, goalSE2);
    const double hR2  = planner.default_heuristic(from, goalR2);

    // Both should equal the Euclidean distance sqrt(9+16)=5
    EXPECT_NEAR(hSE2, 5.0, 1e-6);
    EXPECT_NEAR(hR2, 5.0, 1e-6);
    EXPECT_NEAR(hSE2, hR2, 1e-9);
}

TEST(Heuristic, R2IgnoresHeading)
{
    mpp::TPS_Astar planner;

    const auto goal = makeGoalR2(3.0, 0.0);

    const double h0   = planner.default_heuristic(makeState(0, 0, 0), goal);
    const double h90  = planner.default_heuristic(makeState(0, 0, 90), goal);
    const double h180 = planner.default_heuristic(makeState(0, 0, 180), goal);

    // R(2) heuristic does not depend on the robot's current heading.
    EXPECT_NEAR(h0, h90, 1e-9);
    EXPECT_NEAR(h0, h180, 1e-9);
}

TEST(Heuristic, SE2IncludesHeadingCost)
{
    // With angle weight > 0, a robot facing away from the goal should have
    // higher heuristic than one aligned toward it.
    mpp::TPS_Astar planner;
    planner.params_.SE2_metricAngleWeight    = 1.0;
    planner.params_.heuristic_heading_weight = 0.5;

    const auto goal = makeGoalSE2(3.0, 0.0, 0.0);

    // Robot at (0,0) facing toward goal (phi=0)
    const double hAligned = planner.default_heuristic(makeState(0, 0, 0), goal);
    // Robot at (0,0) facing 180deg away from goal
    const double hOpposite =
        planner.default_heuristic(makeState(0, 0, 180), goal);

    EXPECT_GT(hOpposite, hAligned);
}
