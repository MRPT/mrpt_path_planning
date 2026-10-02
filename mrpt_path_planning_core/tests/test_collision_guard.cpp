/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for CollisionGuard.
 *
 * Verified behaviors:
 *  - Signed distance to the footprint.
 *  - Obstacles ahead limit forward motion but not reversing, and vice versa.
 *  - Arcs: only obstacles on the swept arc limit the command; the curvature
 *    of a limited command is kept.
 *  - In-place rotations are limited by obstacles swept by the footprint.
 *  - Missing / stale obstacle data stops the robot.
 *  - A robot left within the margin of an obstacle can still move away.
 *  - Closed loop: a vehicle with reaction delay and finite braking, driven by
 *    random commands through the guard, never collides.
 */

#include <gtest/gtest.h>
#include <mpp/algos/CollisionGuard.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPose2D.h>

#include <random>

namespace
{
// 1.0 x 0.6 m footprint, origin 0.2 m from the rear:
mpp::RobotShape boxShape()
{
    mrpt::math::TPolygon2D poly;
    poly.emplace_back(-0.2, -0.3);
    poly.emplace_back(0.8, -0.3);
    poly.emplace_back(0.8, 0.3);
    poly.emplace_back(-0.2, 0.3);
    return poly;
}

mpp::CollisionGuard makeGuard()
{
    mpp::CollisionGuard g;
    g.params.max_decel         = 1.0;
    g.params.max_ang_decel     = 2.0;
    g.params.reaction_time     = 0.1;
    g.params.margin            = 0.05;
    g.params.max_obstacles_age = 0.5;
    g.setRobotShape(boxShape());
    return g;
}

// A wall of points along x = wallX, y in [-2,2]:
std::vector<mrpt::math::TPoint2D> wallAtX(double wallX)
{
    std::vector<mrpt::math::TPoint2D> pts;
    for (double y = -2.0; y <= 2.0; y += 0.02) { pts.emplace_back(wallX, y); }
    return pts;
}

const auto kNow = mrpt::Clock::now();
}  // namespace

TEST(CollisionGuard, SignedDistance)
{
    const auto g = makeGuard();
    EXPECT_NEAR(g.signedDistance({0.0, 0.0}), -0.2, 1e-9);
    EXPECT_NEAR(g.signedDistance({1.0, 0.0}), 0.2, 1e-9);
    EXPECT_NEAR(g.signedDistance({0.3, 0.5}), 0.2, 1e-9);
    EXPECT_NEAR(g.signedDistance({0.8, 0.3}), 0.0, 1e-9);

    mpp::CollisionGuard gc;
    gc.setRobotShape(mpp::robot_radius_t{0.5});
    EXPECT_NEAR(gc.signedDistance({1.0, 0.0}), 0.5, 1e-9);
    EXPECT_NEAR(gc.signedDistance({0.0, 0.2}), -0.3, 1e-9);
}

TEST(CollisionGuard, WallAhead)
{
    auto g = makeGuard();
    g.setObstacles(wallAtX(3.0), kNow);

    // Front at 0.8 m, wall at 3.0 m, margin 0.05 => ~2.15 m free:
    const auto r = g.filter(3.0, 0.0, kNow);
    EXPECT_TRUE(r.limited);
    EXPECT_NEAR(r.free_travel, 2.15, 0.03);
    EXPECT_GT(r.v, 0.5);
    EXPECT_LT(r.v, 3.0);
    // The safe speed can stop in the free distance:
    const double tR = g.params.reaction_time;
    EXPECT_LE(
        r.v * tR + r.v * r.v / (2 * g.params.max_decel), r.free_travel + 1e-6);

    // Slow enough: untouched.
    const auto r2 = g.filter(0.5, 0.0, kNow);
    EXPECT_FALSE(r2.limited);
    EXPECT_DOUBLE_EQ(r2.v, 0.5);

    // Reversing away from the wall: untouched, even if fast.
    const auto r3 = g.filter(-3.0, 0.0, kNow);
    EXPECT_FALSE(r3.limited);
    EXPECT_DOUBLE_EQ(r3.v, -3.0);
}

TEST(CollisionGuard, WallBehind)
{
    auto g = makeGuard();
    g.setObstacles(wallAtX(-1.0), kNow);

    const auto r = g.filter(-2.0, 0.0, kNow);
    EXPECT_TRUE(r.limited);
    EXPECT_LT(r.v, 0.0);
    EXPECT_GT(r.v, -2.0);

    EXPECT_FALSE(g.filter(2.0, 0.0, kNow).limited);
}

TEST(CollisionGuard, ArcKeepsCurvature)
{
    auto g = makeGuard();
    // A single obstacle ahead-left, on a left turn of radius 2 m:
    // circle centered at (0,2): point at angle 60 deg from start.
    const double R  = 2.0;
    const double th = M_PI / 3;
    g.setObstacles({{R * std::sin(th), R - R * std::cos(th)}}, kNow);

    const double v = 2.5;
    const auto   r = g.filter(v, v / R, kNow);
    EXPECT_TRUE(r.limited);
    EXPECT_GT(r.v, 0.0);
    EXPECT_LT(r.v, v);
    EXPECT_NEAR(r.omega / r.v, 1.0 / R, 1e-9);

    // Going straight, the same obstacle (y = 1 m) is clear of the footprint:
    EXPECT_FALSE(g.filter(v, 0.0, kNow).limited);
    // Turning right, too:
    EXPECT_FALSE(g.filter(v, -v / R, kNow).limited);
}

TEST(CollisionGuard, RotationInPlace)
{
    auto g = makeGuard();
    // Obstacle in front-left, reached by the front-left corner when rotating
    // to the left (CCW), but not when rotating CW:
    const double rc = std::hypot(0.8, 0.3);
    const double a  = std::atan2(0.3, 0.8) + 0.3;
    g.setObstacles(
        {{(rc + 0.01) * std::cos(a), (rc + 0.01) * std::sin(a)}}, kNow);

    const auto r = g.filter(0.0, 2.0, kNow);
    EXPECT_TRUE(r.limited);
    EXPECT_DOUBLE_EQ(r.v, 0.0);
    EXPECT_LT(r.omega, 2.0);

    // CW: the rear corners (radius ~0.36) never reach it; front-right
    // corner sweeps through it only after ~2 rad, beyond the stop angle.
    EXPECT_FALSE(g.filter(0.0, -1.0, kNow).limited);
}

TEST(CollisionGuard, StaleOrMissingData)
{
    auto g = makeGuard();
    // No data at all:
    auto r = g.filter(1.0, 0.0, kNow);
    EXPECT_TRUE(r.stale);
    EXPECT_EQ(r.v, 0.0);
    EXPECT_EQ(r.omega, 0.0);

    // Old data:
    g.setObstacles(std::vector<mrpt::math::TPoint2D>{}, kNow);
    r = g.filter(
        1.0, 0.0, mrpt::Clock::fromDouble(mrpt::Clock::toDouble(kNow) + 1.0));
    EXPECT_TRUE(r.stale);
    EXPECT_EQ(r.v, 0.0);

    // Fresh, empty data: free to go.
    r = g.filter(1.0, 0.0, kNow);
    EXPECT_FALSE(r.stale);
    EXPECT_FALSE(r.limited);

    // Stopping is never modified:
    EXPECT_FALSE(g.filter(0.0, 0.0, mrpt::Clock::fromDouble(1e9)).limited);

    // Never any obstacle data, even with the age check disabled: stop.
    {
        auto g2                     = makeGuard();
        g2.params.max_obstacles_age = 0;
        const auto r2               = g2.filter(1.0, 0.0, kNow);
        EXPECT_TRUE(r2.stale);
        EXPECT_EQ(r2.v, 0.0);
    }

    // Disabled check:
    g.params.max_obstacles_age = 0;
    r                          = g.filter(
                                 1.0, 0.0, mrpt::Clock::fromDouble(mrpt::Clock::toDouble(kNow) + 10.0));
    EXPECT_FALSE(r.stale);
}

TEST(CollisionGuard, CanLeaveWhenTooClose)
{
    auto g = makeGuard();
    // Wall parallel to the robot, 2 cm from its left side (within margin):
    std::vector<mrpt::math::TPoint2D> pts;
    for (double x = -3.0; x <= 3.0; x += 0.02) { pts.emplace_back(x, 0.32); }
    g.setObstacles(pts, kNow);

    // Driving along the wall, or away from it, is allowed:
    EXPECT_FALSE(g.filter(1.0, 0.0, kNow).limited);
    EXPECT_FALSE(g.filter(-1.0, 0.0, kNow).limited);
    EXPECT_FALSE(g.filter(1.0, -0.5, kNow).limited);
    // Turning into it is not:
    EXPECT_TRUE(g.filter(1.0, 0.5, kNow).limited);
}

TEST(CollisionGuard, ClosedLoopNeverCollides)
{
    std::mt19937                     rng(1234);
    std::uniform_real_distribution<> U(-1.0, 1.0);

    const double dt = 0.05;  // control period [s]

    int episodesLimited = 0;
    for (int episode = 0; episode < 200; episode++)
    {
        auto g = makeGuard();
        // Commands are held for one period, and take effect one period later:
        g.params.reaction_time = 2 * dt;

        // Random obstacles in the world, not within the initial footprint:
        std::vector<mrpt::math::TPoint2D> world;
        while (world.size() < 40)
        {
            const mrpt::math::TPoint2D p(6 * U(rng), 6 * U(rng));
            if (g.signedDistance(p) > 0.2) { world.push_back(p); }
        }

        // Constant desired command per episode:
        const double vDes = 2.5 * U(rng);
        const double wDes = 1.5 * U(rng);

        mrpt::math::TPose2D pose(0, 0, 0);
        double              vAct       = 0;
        double              wAct       = 0;
        double              vPrevCmd   = 0;
        double              wPrevCmd   = 0;
        bool                wasLimited = false;

        for (int step = 0; step < 200; step++)
        {
            // Obstacles in the current robot frame:
            std::vector<mrpt::math::TPoint2D> local;
            for (const auto& p : world)
            {
                const auto q = pose.inverseComposePoint(p);
                local.emplace_back(q.x, q.y);
                ASSERT_GT(g.signedDistance({q.x, q.y}), 0.0)
                    << "Collision! episode=" << episode << " step=" << step
                    << " vDes=" << vDes << " wDes=" << wDes;
            }
            g.setObstacles(local, kNow);
            const auto cmd =
                g.filter(vDes, wDes, kNow, mrpt::math::TTwist2D(vAct, 0, wAct));
            wasLimited = wasLimited || cmd.limited;

            // Vehicle: executes the previous command (one period of delay),
            // with the guard's braking capabilities (accelerating is also
            // bounded, as in a real vehicle):
            auto approach = [](double cur, double target, double maxStep)
            { return cur + std::clamp(target - cur, -maxStep, maxStep); };
            // A stop command brakes along the current arc (as a car keeps its
            // steering, or a velocity smoother scales v and w alike):
            const bool stopCmd =
                std::abs(vPrevCmd) < 1e-3 && std::abs(wPrevCmd) < 1e-3;
            const double curvAct = std::abs(vAct) > 1e-3 ? wAct / vAct : 0.0;
            const double vNew =
                approach(vAct, vPrevCmd, g.params.max_decel * dt);
            if (std::abs(vPrevCmd) > 1e-3)
            {
                wAct = vNew * (wPrevCmd / vPrevCmd);
            }
            else if (stopCmd && std::abs(vAct) > 1e-3)
            {
                wAct = vNew * curvAct;
            }
            else
            {
                wAct = approach(wAct, wPrevCmd, g.params.max_ang_decel * dt);
            }
            vAct     = vNew;
            vPrevCmd = cmd.v;
            wPrevCmd = cmd.omega;

            // Exact unicycle integration:
            if (std::abs(wAct) < 1e-9)
            {
                pose.x += vAct * dt * std::cos(pose.phi);
                pose.y += vAct * dt * std::sin(pose.phi);
            }
            else
            {
                const double R = vAct / wAct;
                pose.x +=
                    R * (std::sin(pose.phi + wAct * dt) - std::sin(pose.phi));
                pose.y -=
                    R * (std::cos(pose.phi + wAct * dt) - std::cos(pose.phi));
                pose.phi += wAct * dt;
            }
        }
        if (wasLimited) { episodesLimited++; }
    }
    // The test is only meaningful if the guard had to act often:
    EXPECT_GT(episodesLimited, 50);
}
