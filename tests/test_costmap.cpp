/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for CostEvaluatorCostMap.
 *
 * Verified behaviors:
 *  - Cost at the obstacle location is maxCost (no singularity fix: P1-costmap).
 *  - Cost at distance >= preferredClearanceDistance is zero.
 *  - Cost is finite everywhere (no NaN, no Inf).
 *  - Cost decreases monotonically with distance to the obstacle.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/data/MoveEdgeSE2_TPS.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPolygon2D.h>

#include <cmath>

namespace
{

mpp::CostEvaluatorCostMap::Ptr makeMapWithOneObstacle(
    double obstX, double obstY, double maxCost = 1.0,
    double preferredClearance = 0.5, double resolution = 0.05)
{
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(obstX, obstY, 0.0);

    mpp::CostEvaluatorCostMap::Parameters p;
    p.maxCost                    = maxCost;
    p.preferredClearanceDistance = preferredClearance;
    p.resolution                 = resolution;

    return mpp::CostEvaluatorCostMap::FromStaticPointObstacles(*pts, p);
}

// Helper: read cost at (x,y) directly from the internal grid.
double costAt(const mpp::CostEvaluatorCostMap& cm, double x, double y)
{
    const auto&   grid = cm.cost_gridmap();
    const double* cell = grid.cellByPos(x, y);
    if (cell == nullptr)
    {
        return 0.0;  // outside grid -> no cost
    }
    return *cell;
}
}  // namespace

// ---- Tests ------------------------------------------------------------------

TEST(CostMap, NoSingularityAtObstacle)
{
    // Prior to the P1-costmap fix the formula diverged at d=0.
    // Now cost(d=0) must equal maxCost (finite).
    auto         cm = makeMapWithOneObstacle(0.0, 0.0, /*maxCost=*/2.0);
    const double c  = costAt(*cm, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(c)) << "Cost at obstacle must be finite";
    EXPECT_NEAR(c, 2.0, 0.01) << "Cost at obstacle must equal maxCost";
}

TEST(CostMap, ZeroCostBeyondClearance)
{
    constexpr double D  = 0.5;
    auto             cm = makeMapWithOneObstacle(0.0, 0.0, /*maxCost=*/1.0, D);

    // A point clearly beyond the clearance distance should have zero cost.
    const double c = costAt(*cm, 0.0, D + 0.1);
    EXPECT_NEAR(c, 0.0, 1e-9);
}

TEST(CostMap, NoNaNOrInfAnywhere)
{
    auto        cm   = makeMapWithOneObstacle(0.0, 0.0);
    const auto& grid = cm->cost_gridmap();

    size_t checked = 0;
    for (int ix = 0; ix < static_cast<int>(grid.getSizeX()); ++ix)
    {
        for (int iy = 0; iy < static_cast<int>(grid.getSizeY()); ++iy)
        {
            const double* cell = grid.cellByIndex(ix, iy);
            ASSERT_NE(cell, nullptr);
            EXPECT_FALSE(std::isnan(*cell))
                << "NaN at cell (" << ix << "," << iy << ")";
            EXPECT_FALSE(std::isinf(*cell))
                << "Inf at cell (" << ix << "," << iy << ")";
            ++checked;
        }
    }
    EXPECT_GT(checked, 0u) << "Grid should have at least one cell";
}

TEST(CostMap, MonotonicallyDecreasingWithDistance)
{
    constexpr double D  = 0.5;
    auto             cm = makeMapWithOneObstacle(0.0, 0.0, /*maxCost=*/1.0, D);

    // Sample along the +X axis and verify cost is non-increasing.
    double prevCost = costAt(*cm, 0.0, 0.0);
    for (double x = 0.05; x <= D + 0.01; x += 0.05)
    {
        const double c = costAt(*cm, x, 0.0);
        EXPECT_LE(c, prevCost + 1e-9)
            << "Cost should not increase moving away from obstacle (x=" << x
            << ")";
        prevCost = c;
    }
}

// Single-pose edge whose reference point (stateFrom.pose) is `pose`, so that
// operator() evaluates the (footprint) cost exactly at `pose`.
namespace
{
mpp::MoveEdgeSE2_TPS makeSinglePoseEdge(const mrpt::math::TPose2D& pose)
{
    mpp::MoveEdgeSE2_TPS e;
    e.stateFrom.pose    = pose;
    e.estimatedExecTime = 1.0;
    e.interpolatedPath[0.0] =
        mrpt::math::TPose2D(0, 0, 0);  // relative to `from`
    return e;
}
}  // namespace

TEST(CostMap, ShapeSamplesEmptyByDefault)
{
    // No robot shape given -> legacy origin-only sampling.
    auto cm = makeMapWithOneObstacle(0.0, 0.0);
    EXPECT_TRUE(cm->shape_samples().empty());
}

TEST(CostMap, FootprintAwareVsOriginOnly)
{
    // Obstacle at the origin; clearance zone of 1 m.
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(0.0, 0.0, 0.0);

    mpp::CostEvaluatorCostMap::Parameters p;
    p.maxCost                    = 1.0;
    p.preferredClearanceDistance = 1.0;
    p.resolution                 = 0.05;

    // Robot footprint: a rectangle extending forward (+x) 1 m ahead of the
    // reference point, half-width 0.3 m.
    mrpt::math::TPolygon2D shape;
    shape.emplace_back(0.0, -0.3);
    shape.emplace_back(1.0, -0.3);
    shape.emplace_back(1.0, 0.3);
    shape.emplace_back(0.0, 0.3);

    auto cmOrigin =
        mpp::CostEvaluatorCostMap::FromStaticPointObstacles(*pts, p);
    auto cmShape = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
        *pts, p, std::nullopt, mpp::RobotShape(shape));

    EXPECT_FALSE(cmShape->shape_samples().empty());

    // Reference point 1.2 m from the obstacle (beyond the 1 m clearance), but
    // facing the obstacle (yaw=pi) so the footprint's front edge reaches to
    // ~0.2 m from it.
    const auto edge = makeSinglePoseEdge({1.2, 0.0, M_PI});

    const double cOrigin = (*cmOrigin)(edge);
    const double cShape  = (*cmShape)(edge);

    EXPECT_NEAR(cOrigin, 0.0, 1e-9)
        << "Origin-only: reference point is beyond the clearance distance";
    EXPECT_GT(cShape, 0.4)
        << "Footprint-aware: the front edge is well inside the clearance zone";
    EXPECT_LE(cShape, p.maxCost + 1e-9);
}

TEST(CostMap, FootprintAwareFarAwayIsZero)
{
    // Same footprint, but far enough that not even the footprint reaches the
    // clearance zone -> zero cost.
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    pts->insertPoint(0.0, 0.0, 0.0);

    mpp::CostEvaluatorCostMap::Parameters p;
    p.maxCost                    = 1.0;
    p.preferredClearanceDistance = 1.0;
    p.resolution                 = 0.05;

    mrpt::math::TPolygon2D shape;
    shape.emplace_back(0.0, -0.3);
    shape.emplace_back(1.0, -0.3);
    shape.emplace_back(1.0, 0.3);
    shape.emplace_back(0.0, 0.3);

    auto cmShape = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
        *pts, p, std::nullopt, mpp::RobotShape(shape));

    // Facing away (+x), reference point at x=2.5: nearest footprint point is at
    // x=2.5, well beyond the 1 m clearance.
    const auto   edge   = makeSinglePoseEdge({2.5, 0.0, 0.0});
    const double cShape = (*cmShape)(edge);
    EXPECT_NEAR(cShape, 0.0, 1e-9);
}

TEST(CostMap, QuadraticDecay)
{
    // Cost = maxCost * (1 - d/D)^2  (smooth quadratic, the post-fix formula)
    constexpr double maxCost = 1.0;
    constexpr double D       = 0.5;
    auto cm = makeMapWithOneObstacle(0.0, 0.0, maxCost, D, /*resolution=*/0.01);

    for (double d = 0.0; d < D; d += 0.1)
    {
        const double measured = costAt(*cm, d, 0.0);
        const double nd       = 1.0 - d / D;
        const double expected = maxCost * nd * nd;
        EXPECT_NEAR(measured, expected, 0.02)
            << "Quadratic formula mismatch at d=" << d;
    }
}
