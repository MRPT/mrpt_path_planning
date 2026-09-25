/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Soundness of the precomputed PTG collision grid.
 *
 * For random obstacle points and every trajectory of a C-PTG, the free
 * distance read from the collision grid must never exceed the true distance at
 * which the continuously-swept footprint (optionally grown by the configured
 * clearance) first touches the point. The ground truth is computed by brute
 * force, sweeping the exact, non-convex polygon along a dense interpolation
 * of the trajectory.
 *
 * Verified behaviors:
 *  - Soundness with zero clearance, for a non-convex (L-shaped) footprint.
 *  - Soundness with a nonzero clearance.
 *  - Obstacles beyond the trajectory reference distance but within reach of
 *    the footprint are not ignored.
 */

#include <gtest/gtest.h>
#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

namespace
{
// L-shaped (non-convex) footprint, 6 vertices.
std::string ptgConfig(double clearance)
{
    return std::string(R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0

PTG_COUNT = 1

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 2.0
PTG0_num_paths   = 21
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 90.0
PTG0_K           = +1.0
)cfg") +
           "PTG0_clearance = " + std::to_string(clearance) + R"cfg(

RobotModel_shape2D_xs = -0.20 0.35 0.35 0.05 0.05 -0.20
RobotModel_shape2D_ys = -0.20 -0.20 0.05 0.05 0.20 0.20
)cfg";
}

mrpt::math::TPolygon2D footprintAt(
    const mrpt::math::TPolygon2D& shape, const mrpt::math::TPose2D& p)
{
    mrpt::math::TPolygon2D out;
    for (const auto& v : shape)
    {
        out.emplace_back(
            p.x + std::cos(p.phi) * v.x - std::sin(p.phi) * v.y,
            p.y + std::sin(p.phi) * v.x + std::cos(p.phi) * v.y);
    }
    return out;
}

double pointPolygonDistance(
    const mrpt::math::TPoint2D& q, const mrpt::math::TPolygon2D& poly)
{
    if (poly.contains(q)) { return 0; }
    double       d = std::numeric_limits<double>::max();
    const size_t N = poly.size();
    for (size_t i = 0; i < N; i++)
    {
        const auto&  a    = poly[i];
        const auto&  b    = poly[(i + 1) % N];
        const double abx  = b.x - a.x;
        const double aby  = b.y - a.y;
        const double len2 = abx * abx + aby * aby;
        double       t    = ((q.x - a.x) * abx + (q.y - a.y) * aby) / len2;
        t                 = std::clamp(t, 0.0, 1.0);
        d = std::min(d, std::hypot(q.x - a.x - t * abx, q.y - a.y - t * aby));
    }
    return d;
}

// Returns the number of (point, trajectory) pairs actually in contact, so
// callers can check the test is not vacuous.
size_t checkSoundness(double clearance, size_t nPoints)
{
    mrpt::config::CConfigFileMemory cfg(ptgConfig(clearance));
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    EXPECT_EQ(trs.ptgs.size(), 1U);
    const auto& ptg = *trs.ptgs.at(0);

    const auto shape = std::get<mrpt::math::TPolygon2D>(trs.robotShape);

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    const double range     = ptg.getRefDistance() + 0.6;
    const int    kSubSteps = 4;

    size_t nContacts = 0;
    for (size_t i = 0; i < nPoints; i++)
    {
        const mrpt::math::TPoint2D o(
            rng.drawUniform(-range, range), rng.drawUniform(-range, range));

        // Skip points already touching the footprint at the origin: those are
        // handled by the PTG collision-behavior policy, not by the grid.
        if (pointPolygonDistance(o, shape) <= clearance + 1e-3) { continue; }

        for (uint16_t k = 0; k < ptg.getPathCount(); k++)
        {
            double gridFree = 0;
            ptg.initTPObstacleSingle(k, gridFree);
            ptg.updateTPObstacleSingle(o.x, o.y, k, gridFree);

            // Brute-force first contact along the densely interpolated path:
            const size_t nSteps           = ptg.getPathStepCount(k);
            double       trueFirstContact = std::numeric_limits<double>::max();
            for (size_t n = 0; n + 1 < nSteps && trueFirstContact > 1e9; n++)
            {
                const auto p0 = ptg.getPathPose(k, n);
                const auto p1 = ptg.getPathPose(k, n + 1);
                for (int s = 0; s < kSubSteps; s++)
                {
                    const double              t = double(s) / kSubSteps;
                    const mrpt::math::TPose2D p(
                        p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y),
                        p0.phi + t * mrpt::math::angDistance(p0.phi, p1.phi));
                    if (pointPolygonDistance(o, footprintAt(shape, p)) <=
                        clearance)
                    {
                        trueFirstContact = ptg.getPathDist(k, n);
                        break;
                    }
                }
            }
            if (trueFirstContact > 1e9) { continue; }
            nContacts++;

            EXPECT_LE(gridFree, trueFirstContact + 1e-6)
                << "k=" << k << " obstacle=" << o.asString()
                << " clearance=" << clearance;
        }
    }
    return nContacts;
}
}  // namespace

TEST(CollisionGridSoundness, NonConvexFootprintNoClearance)
{
    const size_t nContacts = checkSoundness(0.0, 150);
    EXPECT_GT(nContacts, 50U);
}

TEST(CollisionGridSoundness, NonConvexFootprintWithClearance)
{
    const size_t nContacts = checkSoundness(0.05, 150);
    EXPECT_GT(nContacts, 50U);
}

TEST(CollisionGridSoundness, ObstacleBeyondRefDistanceIsSeen)
{
    mrpt::config::CConfigFileMemory cfg(ptgConfig(0.0));
    mpp::TrajectoriesAndRobotShape  trs;
    trs.initFromConfigFile(cfg, "SelfDriving");
    const auto& ptg = *trs.ptgs.at(0);

    // Straight-ahead trajectory: the front of the footprint ends 0.35 m past
    // refDistance, so an obstacle 0.2 m beyond it must block that path.
    const uint16_t kStraight = ptg.getPathCount() / 2;
    const double   ox        = ptg.getRefDistance() + 0.2;

    double gridFree = 0;
    ptg.initTPObstacleSingle(kStraight, gridFree);
    ptg.updateTPObstacleSingle(ox, 0.0, kStraight, gridFree);
    EXPECT_LT(gridFree, ptg.getRefDistance());
}
