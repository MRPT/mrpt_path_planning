/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for CostEvaluatorPreferredWaypoint.
 *
 * Verified behaviors:
 *  - No waypoints → cost equals costScale.
 *  - Edge passing through a waypoint → cost strictly reduced.
 *  - Edge far outside influence radius → cost equals costScale.
 *  - Average and max aggregation modes both stay in [0, costScale].
 *  - Replacing the waypoint set updates costs immediately.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>

static mpp::MoveEdgeSE2_TPS makeEdge(
    std::initializer_list<std::pair<double, mrpt::math::TPose2D>> entries)
{
    mpp::MoveEdgeSE2_TPS edge;
    edge.stateFrom.pose = {0, 0, 0};
    for (const auto& [t, p] : entries) edge.interpolatedPath[t] = p;
    return edge;
}

static mpp::CostEvaluatorPreferredWaypoint makeEv(
    double scale, double radius, bool useAvg)
{
    mpp::CostEvaluatorPreferredWaypoint ev;
    ev.params_.costScale               = scale;
    ev.params_.waypointInfluenceRadius = radius;
    ev.params_.useAverageOfPath        = useAvg;
    return ev;
}

// ---------------------------------------------------------------------------

TEST(CostEvaluatorWaypoint, NoWaypoints)
{
    auto ev = makeEv(/*scale=*/1.0, /*radius=*/1.0, /*avg=*/true);

    const auto   edge = makeEdge({{0.0, {0, 0, 0}}, {1.0, {2, 0, 0}}});
    const double cost = ev(edge);

    EXPECT_NEAR(cost, 1.0, 1e-9)
        << "With no waypoints cost must equal costScale";
}

TEST(CostEvaluatorWaypoint, EdgePassesThroughWaypoint)
{
    auto ev = makeEv(/*scale=*/1.0, /*radius=*/0.5, /*avg=*/true);
    ev.setPreferredWaypoints({{1.0, 0.0}});

    // Middle sample sits exactly on the waypoint → cost at that pose = 0.
    const auto edge = makeEdge(
        {{0.0, {0.0, 0, 0}},
         {0.5, {1.0, 0, 0}},  // hits waypoint (1,0)
         {1.0, {2.0, 0, 0}}});

    const double cost = ev(edge);
    EXPECT_LT(cost, 1.0) << "Cost must be reduced when path crosses waypoint";
    EXPECT_GE(cost, 0.0) << "Cost must be non-negative";
}

TEST(CostEvaluatorWaypoint, EdgeFarFromWaypoint)
{
    auto ev = makeEv(/*scale=*/1.0, /*radius=*/0.5, /*avg=*/true);
    ev.setPreferredWaypoints({{10.0, 10.0}});

    const auto   edge = makeEdge({{0.0, {0, 0, 0}}, {1.0, {1, 0, 0}}});
    const double cost = ev(edge);

    EXPECT_NEAR(cost, 1.0, 1e-9)
        << "Waypoint outside influence radius must not change cost";
}

TEST(CostEvaluatorWaypoint, UseAverageVsMax)
{
    // Two poses: one at the waypoint (cost=0), one outside (cost=costScale).
    // Average mode returns the mean; max mode returns the maximum per-pose cost.
    // Both must stay in [0, costScale].
    const auto edge = makeEdge(
        {{0.0, {0.0, 0, 0}},   // outside radius → cost = costScale
         {1.0, {1.0, 0, 0}}});  // at waypoint   → cost = 0

    auto evAvg = makeEv(1.0, 0.5, /*avg=*/true);
    evAvg.setPreferredWaypoints({{1.0, 0.0}});

    auto evMax = makeEv(1.0, 0.5, /*avg=*/false);
    evMax.setPreferredWaypoints({{1.0, 0.0}});

    const double costAvg = evAvg(edge);
    const double costMax = evMax(edge);

    EXPECT_GE(costAvg, 0.0);
    EXPECT_LE(costAvg, 1.0);
    EXPECT_GE(costMax, 0.0);
    EXPECT_LE(costMax, 1.0);

    // Average discounts the waypoint hit; max keeps the worst-case pose cost.
    EXPECT_LT(costAvg, costMax);
}

TEST(CostEvaluatorWaypoint, SetPreferredWaypointsUpdates)
{
    auto ev = makeEv(/*scale=*/1.0, /*radius=*/0.5, /*avg=*/true);

    // Edge with one pose at (1,0) — on the waypoint.
    const auto edge = makeEdge({{0.5, {1.0, 0, 0}}});

    ev.setPreferredWaypoints({{1.0, 0.0}});
    EXPECT_LT(ev(edge), 1.0) << "Cost must be reduced with waypoint at (1,0)";

    ev.setPreferredWaypoints({});
    EXPECT_NEAR(ev(edge), 1.0, 1e-9) << "Cost must return to costScale after clearing waypoints";
}
