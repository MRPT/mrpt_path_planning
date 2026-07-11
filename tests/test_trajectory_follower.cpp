/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Unit tests for the pure-pursuit TrajectoryFollower core (no safety yet).
 * Each test closes the loop with a trivial unicycle sim: feed the follower the
 * robot pose as both localization (map) and odometry (identity map->odom),
 * execute the immediate command twist for one control period, repeat.
 */

#include <gtest/gtest.h>
#include <mpp/follow/algos/TrajectoryFollower.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/wrap2pi.h>

#include <chrono>
#include <cmath>

namespace
{
using mrpt::math::TPoint2D;
using mrpt::math::TPose2D;

mrpt::math::TPose2D integrate(const TPose2D& p, double v, double w, double dt)
{
    double x = p.x, y = p.y, phi = p.phi;
    if (std::abs(w) < 1e-6)
    {
        x += v * dt * std::cos(phi);
        y += v * dt * std::sin(phi);
    }
    else
    {
        const double dphi = w * dt;
        const double R    = v / w;
        x += R * (std::sin(phi + dphi) - std::sin(phi));
        y += -R * (std::cos(phi + dphi) - std::cos(phi));
        phi += dphi;
    }
    return {x, y, mrpt::math::wrapToPi(phi)};
}

double distToPolyline(const std::vector<TPoint2D>& pts, const TPoint2D& q)
{
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < pts.size(); i++)
    {
        const auto&  a  = pts[i];
        const auto&  b  = pts[i + 1];
        const double sx = b.x - a.x, sy = b.y - a.y;
        const double len2 = sx * sx + sy * sy;
        double       t =
            len2 > 1e-9 ? ((q.x - a.x) * sx + (q.y - a.y) * sy) / len2 : 0;
        t               = std::clamp(t, 0.0, 1.0);
        const double px = a.x + t * sx, py = a.y + t * sy;
        best = std::min(best, std::hypot(q.x - px, q.y - py));
    }
    return best;
}

mpp::Trajectory polyToTraj(const std::vector<TPoint2D>& pts, double speed)
{
    mpp::Trajectory tr;
    for (std::size_t i = 0; i < pts.size(); i++)
    {
        double heading = 0;
        if (i + 1 < pts.size())
            heading =
                std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        else if (i > 0)
            heading =
                std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
        tr.emplace_back(TPose2D(pts[i].x, pts[i].y, heading), speed);
    }
    return tr;
}

mpp::VehicleLocalizationState mkLoc(const TPose2D& p)
{
    mpp::VehicleLocalizationState s;
    s.valid = true;
    s.pose  = p;
    return s;
}
mpp::VehicleOdometryState mkOdo(const TPose2D& p, double v)
{
    mpp::VehicleOdometryState s;
    s.valid                 = true;
    s.odometry              = p;  // identity map->odom in these tests
    s.odometryVelocityLocal = mrpt::math::TTwist2D(v, 0, 0);
    return s;
}

struct SimResult
{
    bool                reached  = false;
    double              maxCross = 0;
    double              maxSpeed = 0;
    TPose2D             finalPose;
    int                 steps      = 0;
    mpp::FollowerStatus lastStatus = mpp::FollowerStatus::Idle;
};

SimResult simulate(
    mpp::TrajectoryFollower& f, const TPose2D& start,
    const std::vector<TPoint2D>& pathPts, int maxSteps = 2000)
{
    SimResult    r;
    TPose2D      robot = start;
    double       v     = 0;
    const double dt    = f.params.control_period;
    for (int k = 0; k < maxSteps; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        r.lastStatus   = out.status;
        r.maxCross =
            std::max(r.maxCross, distToPolyline(pathPts, {robot.x, robot.y}));
        r.finalPose = robot;
        r.steps     = k;
        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            r.reached = true;
            break;
        }
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        r.maxSpeed    = std::max(r.maxSpeed, tw.vx);
        robot         = integrate(robot, tw.vx, tw.omega, dt);
        v             = tw.vx;
    }
    return r;
}
}  // namespace

// ---------------------------------------------------------------------------

TEST(TrajectoryFollower, IdleWithoutTrajectory)
{
    mpp::TrajectoryFollower f;
    const auto              out = f.step(mkLoc({0, 0, 0}), mkOdo({0, 0, 0}, 0));
    EXPECT_EQ(out.status, mpp::FollowerStatus::Idle);
    EXPECT_TRUE(out.command.empty());
}

TEST(TrajectoryFollower, StraightLineOnPath)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {5, 0}};
    mpp::TrajectoryFollower     f;
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(r.maxCross, 0.05);
    EXPECT_NEAR(r.finalPose.x, 5.0, 0.2);
    EXPECT_NEAR(r.finalPose.y, 0.0, 0.2);
}

TEST(TrajectoryFollower, ConvergesFromLateralOffset)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {8, 0}};
    mpp::TrajectoryFollower     f;
    f.setTrajectory(polyToTraj(pts, 0.5));

    // Start 0.5 m off the line, facing along it.
    const auto r = simulate(f, {0, 0.5, 0}, pts);
    EXPECT_TRUE(r.reached);
    // It must actually rejoin: end essentially on the line.
    EXPECT_LT(distToPolyline(pts, {r.finalPose.x, r.finalPose.y}), 0.1);
    // And never diverge worse than the initial offset.
    EXPECT_LT(r.maxCross, 0.6);
}

TEST(TrajectoryFollower, FollowsCornerPath)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {4, 0}, {4, 4}};
    mpp::TrajectoryFollower     f;
    f.params.max_speed = 0.4;
    f.setTrajectory(polyToTraj(pts, 0.4));

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(r.maxCross, 0.7);  // cuts the corner but stays bounded
    EXPECT_NEAR(r.finalPose.x, 4.0, 0.3);
    EXPECT_NEAR(r.finalPose.y, 4.0, 0.3);
}

TEST(TrajectoryFollower, RespectsSpeedCap)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {6, 0}};
    mpp::TrajectoryFollower     f;
    f.params.max_speed = 1.0;
    f.setTrajectory(
        polyToTraj(pts, 0.2));  // per-point cap well below max_speed

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_LE(r.maxSpeed, 0.2 + 1e-3);
}

TEST(TrajectoryFollower, OffPathExceeded)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {5, 0}};
    mpp::TrajectoryFollower     f;
    f.params.max_cross_track = 1.0;
    f.setTrajectory(polyToTraj(pts, 0.5));

    // Start 2 m off the line -> beyond the cross-track limit on the first step.
    const auto out = f.step(mkLoc({1, 2, 0}), mkOdo({1, 2, 0}, 0));
    EXPECT_EQ(out.status, mpp::FollowerStatus::OffPathExceeded);
    EXPECT_FALSE(out.command.empty());  // still tries to rejoin
}

TEST(TrajectoryFollower, EmitsHorizonChunk)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {5, 0}};
    mpp::TrajectoryFollower     f;
    f.params.horizon       = 1.5;
    f.params.sample_period = 0.1;
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto out = f.step(mkLoc({0, 0, 0}), mkOdo({0, 0, 0}, 0));
    EXPECT_EQ(out.status, mpp::FollowerStatus::Running);
    ASSERT_GE(out.command.points.size(), 2u);
    EXPECT_EQ(out.command.frame_id, "odom");
    EXPECT_NEAR(out.command.points.front().t, 0.0, 1e-9);
    // strictly increasing time stamps
    for (std::size_t i = 1; i < out.command.points.size(); i++)
        EXPECT_GT(out.command.points[i].t, out.command.points[i - 1].t);
}

// --------------------------- predictive safety ----------------------------

TEST(TrajectoryFollower, SafetyInertWithoutObstacles)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {5, 0}};
    mpp::TrajectoryFollower     f;
    f.setRobotShape(mpp::robot_radius_t{0.3});  // shape but no obstacles
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);  // safety layer does nothing without obstacles
}

TEST(TrajectoryFollower, StopsBeforeObstacle)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {6, 0}};
    mpp::TrajectoryFollower     f;
    f.setRobotShape(mpp::robot_radius_t{0.3});
    f.setObstacles(std::vector<TPoint2D>{{3.0, 0.0}});  // right on the path
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_FALSE(r.reached);  // must not drive through the obstacle
    // Stops short of the obstacle (center + radius margin), but did advance.
    EXPECT_GT(r.finalPose.x, 1.5);
    EXPECT_LT(r.finalPose.x, 2.9);
}

TEST(TrajectoryFollower, ResumesAfterObstacleCleared)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {6, 0}};
    mpp::TrajectoryFollower     f;
    f.setRobotShape(mpp::robot_radius_t{0.3});
    f.setObstacles(std::vector<TPoint2D>{{3.0, 0.0}});
    f.setTrajectory(polyToTraj(pts, 0.5));

    // Phase 1: drive into the obstacle field until it stops.
    TPose2D      robot = {0, 0, 0};
    double       v     = 0;
    const double dt    = f.params.control_period;
    for (int k = 0; k < 2000; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        const auto tw  = out.command.points.front().twist;
        robot          = integrate(robot, tw.vx, tw.omega, dt);
        v              = tw.vx;
        if (v < 1e-3 && robot.x > 1.0) break;  // stopped in front of obstacle
    }
    ASSERT_LT(v, 1e-2);
    ASSERT_LT(robot.x, 2.9);

    // Phase 2: obstacle removed -> must resume and reach the goal.
    f.setObstacles(std::vector<TPoint2D>{});
    bool reached = false;
    for (int k = 0; k < 2000; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            reached = true;
            break;
        }
        const auto tw = out.command.points.front().twist;
        robot         = integrate(robot, tw.vx, tw.omega, dt);
        v             = tw.vx;
    }
    EXPECT_TRUE(reached);
    EXPECT_NEAR(robot.x, 6.0, 0.2);
}

TEST(TrajectoryFollower, BlockedAfterTimeout)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {6, 0}};
    mpp::TrajectoryFollower     f;
    f.params.block_timeout = 5.0;
    f.setRobotShape(mpp::robot_radius_t{0.3});
    f.setObstacles(std::vector<TPoint2D>{{3.0, 0.0}});
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto t0 = mrpt::Clock::now();

    // Start already stopped in front of the obstacle.
    auto loc      = mkLoc({2.6, 0, 0});
    loc.timestamp = t0;
    auto out      = f.step(loc, mkOdo({2.6, 0, 0}, 0));
    EXPECT_EQ(out.safety_scale, 0.0);
    EXPECT_NE(out.status, mpp::FollowerStatus::Blocked);  // just started

    // Same obstacle, timestamp advanced past the block timeout.
    loc.timestamp = t0 + std::chrono::milliseconds(6000);
    out           = f.step(loc, mkOdo({2.6, 0, 0}, 0));
    EXPECT_EQ(out.status, mpp::FollowerStatus::Blocked);
}

TEST(TrajectoryFollower, IgnoresObstacleOffPath)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {6, 0}};
    mpp::TrajectoryFollower     f;
    f.setRobotShape(mpp::robot_radius_t{0.3});
    // Obstacle well clear of the swept footprint corridor.
    f.setObstacles(std::vector<TPoint2D>{{3.0, 2.0}});
    f.setTrajectory(polyToTraj(pts, 0.5));

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_NEAR(r.finalPose.x, 6.0, 0.2);
}

TEST(TrajectoryFollower, ParamsYamlRoundTrip)
{
    mpp::TrajectoryFollower::Parameters p;
    p.max_speed     = 0.33;
    p.lookahead_max = 2.0;
    p.horizon       = 2.5;
    p.stop_distance = 0.42;
    p.slow_distance = 1.75;
    const auto y    = p.as_yaml();
    const auto p2   = mpp::TrajectoryFollower::Parameters::FromYAML(y);
    EXPECT_NEAR(p2.max_speed, 0.33, 1e-9);
    EXPECT_NEAR(p2.lookahead_max, 2.0, 1e-9);
    EXPECT_NEAR(p2.horizon, 2.5, 1e-9);
    EXPECT_NEAR(p2.stop_distance, 0.42, 1e-9);
    EXPECT_NEAR(p2.slow_distance, 1.75, 1e-9);
}
