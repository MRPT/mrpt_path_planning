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
#include <mpp/algos/TrajectoryFollower.h>
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

// Regression: the robot starts ahead of (not on) a straight reference path
// whose waypoints all carry the *same* heading as the robot's own start
// heading, with the path itself running the opposite way in position -- a
// straight reverse where every waypoint's phi is left at the "facing"
// direction rather than the travel direction (e.g. a plan whose points only
// ever carry one constant orientation). The robot, starting at the origin
// facing +x, must back straight into it (drive -x), not drive forward.
TEST(TrajectoryFollower, ReversesOnStraightLineBehindRobot)
{
    const std::vector<TPoint2D> pts = {
        {-1.0, 0.0}, {-1.5, 0.0}, {-2.0, 0.0}, {-2.5, 0.0}, {-3.0, 0.0}};
    mpp::Trajectory tr;
    for (const auto& p : pts) tr.emplace_back(TPose2D(p.x, p.y, 0.0), 0.5);

    mpp::TrajectoryFollower f;
    f.setTrajectory(tr);

    const auto r = simulate(f, {0, 0, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(r.finalPose.x, 0.0) << "must drive backward (-x), not forward";
    EXPECT_NEAR(r.finalPose.x, -3.0, 0.3);
}

// Same straight reverse, but the robot starts laterally offset and must
// steer (nonzero curvature) while already committed to reverse gear -- a
// combination the other straight-reverse tests above don't exercise (they
// have zero curvature need throughout).
TEST(TrajectoryFollower, ReversesWithLateralOffsetNeedingCorrection)
{
    const std::vector<TPoint2D> pts = {
        {-1.0, 0.0}, {-1.5, 0.0}, {-2.0, 0.0}, {-2.5, 0.0}, {-3.0, 0.0}};
    mpp::Trajectory tr;
    for (const auto& p : pts) tr.emplace_back(TPose2D(p.x, p.y, 0.0), 0.5);

    mpp::TrajectoryFollower f;
    f.setTrajectory(tr);

    const auto r = simulate(f, {0, 0.5, 0}, pts);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(r.finalPose.x, 0.0) << "must drive backward (-x), not forward";
    EXPECT_NEAR(r.finalPose.x, -3.0, 0.3);

    // Repeated step() calls once parked at the goal (a real node timer keeps
    // calling step() every cycle regardless of status) must not drift.
    TPose2D robot = r.finalPose;
    double  v     = 0;
    for (int k = 0; k < 20; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        double     vx  = out.command.points.empty()
                             ? 0.0
                             : out.command.points.front().twist.vx;
        double     om  = out.command.points.empty()
                             ? 0.0
                             : out.command.points.front().twist.omega;
        robot          = integrate(robot, vx, om, f.params.control_period);
        v              = vx;
    }
    EXPECT_LT(robot.x, 0.0)
        << "must not drift forward after arrival on repeated step() calls";
}

// Mimics a live caller that re-publishes the same reference path every
// control cycle until superseded (a common pattern for a node driving this
// follower from a periodically-republished planner output).
TEST(TrajectoryFollower, ReversesOnStraightLineRepublished)
{
    const std::vector<TPoint2D> pts = {
        {-1.0, 0.0}, {-1.5, 0.0}, {-2.0, 0.0}, {-2.5, 0.0}, {-3.0, 0.0}};
    mpp::Trajectory tr;
    for (const auto& p : pts) tr.emplace_back(TPose2D(p.x, p.y, 0.0), 0.5);

    mpp::TrajectoryFollower f;
    f.params.max_speed       = 0.5;
    f.params.max_accel       = 0.5;
    f.params.max_decel       = 0.7;
    f.params.min_turn_radius = 0.4;

    TPose2D robot = {0, 0, 0};
    double  v     = 0;
    for (int k = 0; k < 200; k++)
    {
        f.setTrajectory(tr);  // re-published every cycle
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        robot = integrate(robot, tw.vx, tw.omega, f.params.control_period);
        v     = tw.vx;
    }
    EXPECT_LT(robot.x, 0.0) << "must drive backward (-x), not forward";
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

// A differential-drive A* entry path that backs the robot into a row pose
// (reproduced from a simulation run of a robot approaching a row entrance).
// The path loops back on itself, so its *forward* leg passes
// ~0.16 m from the goal *position* while the robot still points ~90 deg away
// from the goal *heading*; a reverse cusp near the end is what swings the nose
// round to the goal heading. Regression: a Euclidean-only arrival test latches
// "reached" on that forward-leg near-pass and stops the robot ~0.3 m short at
// ~75 deg heading error, never running the terminal maneuver. The follower
// must instead drive the whole path and seat the final heading.
namespace
{
mpp::Trajectory entryManeuverTraj(double speed)
{
    // {x, y, heading[deg]} per knot, goal (last) heading = -152.4 deg.
    const double knots[][3] = {
        {2.568, 2.936, -62.0},  {2.610, 2.845, -68.6},  {2.641, 2.750, -75.2},
        {2.661, 2.652, -81.9},  {2.669, 2.553, -88.5},  {2.677, 2.453, -81.9},
        {2.697, 2.355, -75.2},  {2.728, 2.260, -68.6},  {2.770, 2.169, -62.0},
        {2.833, 2.092, -39.8},  {2.920, 2.044, -17.7},  {3.007, 1.997, -39.1},
        {3.071, 1.921, -60.5},  {3.104, 1.827, -81.9},  {3.099, 1.728, -103.3},
        {3.137, 1.820, -121.7}, {3.202, 1.895, -140.2}, {3.281, 1.957, -143.8},
        {3.363, 2.013, -147.5}, {3.236, 1.924, -152.4}};
    mpp::Trajectory tr;
    for (const auto& k : knots)
        tr.emplace_back(TPose2D(k[0], k[1], mrpt::DEG2RAD(k[2])), speed);
    return tr;
}
}  // namespace

TEST(TrajectoryFollower, SelfApproachingEntryPathSeatsFinalHeading)
{
    const auto            tr = entryManeuverTraj(0.3);
    std::vector<TPoint2D> pts;
    for (std::size_t i = 0; i < tr.size(); i++)
        pts.push_back({tr[i].pose.x, tr[i].pose.y});

    mpp::TrajectoryFollower f;
    f.params.max_speed       = 0.3;
    f.params.min_turn_radius = 0.0;  // unicycle sim: can trace the tight cusp

    // Direct gate check: a single step() with the robot sitting on the forward
    // leg (knot 12), 0.16 m from the goal *position* but far from the path end
    // in arc-length, must not be mistaken for arrival.
    f.setTrajectory(tr);
    const TPose2D onForwardLeg{3.071, 1.921, mrpt::DEG2RAD(-60.5)};
    const auto gateOut = f.step(mkLoc(onForwardLeg), mkOdo(onForwardLeg, 0.3));
    EXPECT_NE(gateOut.status, mpp::FollowerStatus::ReachedGoal)
        << "must not arrive on a forward-leg near-pass of a loop-back path";

    // Full closed-loop drive: reach the goal position AND heading.
    f.setTrajectory(tr);  // reset progress
    const auto r =
        simulate(f, TPose2D(2.568, 2.936, mrpt::DEG2RAD(-62.0)), pts, 8000);
    EXPECT_TRUE(r.reached);
    EXPECT_NEAR(r.finalPose.x, 3.236, 0.25);
    EXPECT_NEAR(r.finalPose.y, 1.924, 0.25);
    const double headErrDeg = std::abs(mrpt::RAD2DEG(
        mrpt::math::wrapToPi(r.finalPose.phi - mrpt::DEG2RAD(-152.4))));
    EXPECT_LT(headErrDeg, 25.0)
        << "stopped at heading " << mrpt::RAD2DEG(r.finalPose.phi)
        << " deg (goal -152.4): the terminal maneuver was not executed";
}

// --------------------------------------------------------------------------
// Regression: a short reverse leg (reproduces a "weird turn, could not
// finish" hang seen on a real deployment). The planner backs the robot
// ~0.55 m into a tight goal: start and goal body headings are nearly aligned
// (~ -153 deg), so it should be an almost-straight reverse. The final
// reference pose carries the GOAL BODY heading (opposite the travel direction,
// as real differential-drive A* waypoints do), not the last-segment travel
// direction.
namespace
{
// Build a reverse trajectory: polyline points with each point's heading
// linearly interpolated (by index, not arc-length) between the start and
// goal body headings, matching a real differential-drive planner's waypoint
// convention -- a genuinely reversing vehicle's heading evolves smoothly
// near-constant over a short leg like this one, it does not tangent-track
// its own travel direction (the tangent-to-next-point convention `polyToTraj`
// uses for its *own* purpose is a good stand-in for a forward leg, but
// reports the *opposite* of a real reversing vehicle's heading, and would
// misidentify this leg's own gear).
mpp::Trajectory revTraj(
    const std::vector<TPoint2D>& pts, double startHeading, double goalHeading,
    double speed)
{
    mpp::Trajectory tr;
    const auto      n = pts.size();
    for (std::size_t i = 0; i < n; i++)
    {
        const double t = n > 1 ? static_cast<double>(i) / (n - 1) : 0.0;
        const double heading =
            startHeading + t * mrpt::math::wrapToPi(goalHeading - startHeading);
        tr.emplace_back(TPose2D(pts[i].x, pts[i].y, heading), speed);
    }
    return tr;
}

// Verbose closed-loop sim (unicycle), printing a trace so the follower's intent
// is visible while iterating on a fix.
SimResult simulateVerbose(
    mpp::TrajectoryFollower& f, const TPose2D& start,
    const std::vector<TPoint2D>& pathPts, const TPose2D& goal,
    double maxCurv = 0.0, int maxSteps = 4000)
{
    SimResult    r;
    TPose2D      robot = start;
    double       v     = 0;
    const double dt    = f.params.control_period;
    fprintf(
        stderr, "%5s %8s %8s %8s | %8s %8s %8s %8s %6s\n", "step", "x", "y",
        "yaw", "v", "omega", "dGoal", "hErrDg", "stat");
    for (int k = 0; k < maxSteps; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        r.lastStatus   = out.status;
        r.maxCross =
            std::max(r.maxCross, distToPolyline(pathPts, {robot.x, robot.y}));
        r.finalPose = robot;
        r.steps     = k;

        const double dGoal = std::hypot(robot.x - goal.x, robot.y - goal.y);
        double       vx = 0, om = 0;
        if (!out.command.points.empty())
        {
            vx = out.command.points.front().twist.vx;
            om = out.command.points.front().twist.omega;
        }
        if (k % 10 == 0 || out.status == mpp::FollowerStatus::ReachedGoal)
            fprintf(
                stderr, "%5d %8.3f %8.3f %8.1f | %8.3f %8.3f %8.3f %8.1f %6d\n",
                k, robot.x, robot.y, mrpt::RAD2DEG(robot.phi), vx, om, dGoal,
                mrpt::RAD2DEG(out.heading_err), static_cast<int>(out.status));

        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            r.reached = true;
            break;
        }
        if (out.command.points.empty()) break;
        r.maxSpeed = std::max(r.maxSpeed, vx);
        // Ackermann steering limit: curvature |omega/v| capped at maxCurv, so
        // an in-place spin (omega large, v ~ 0) is physically impossible (a
        // car-like robot must roll to turn). maxCurv <= 0 => ideal unicycle.
        if (maxCurv > 0.0)
        {
            const double maxOm = std::abs(vx) * maxCurv;
            om                 = std::clamp(om, -maxOm, maxOm);
        }
        robot = integrate(robot, vx, om, dt);
        v     = vx;
    }
    return r;
}
}  // namespace

TEST(TrajectoryFollower, ShortReverseEntryLiveCase)
{
    // Captured from a real robot (map frame): start pose and a goal
    // standoff pose, a ~0.55 m reverse with aligned headings.
    const TPose2D start(1.001, 0.553, mrpt::DEG2RAD(-152.7));
    const TPose2D goal(1.545, 0.621, mrpt::DEG2RAD(-153.0));

    // The REAL 11 A* waypoints captured live (make_plan_from_to,
    // obstacle-free). Note the cusps: the positions overshoot to (1.636,0.701)
    // then reverse back to (1.477,0.581) before the goal -- a
    // differential-drive maneuver with direction reversals, not a straight
    // reverse.
    const std::vector<TPoint2D> pts = {
        {1.001, 0.553},
        {1.0946131281188594, 0.5874501963198541},
        {1.1937828734685965, 0.5982016015376657},
        {1.2936350310744624, 0.6024873565384339},
        {1.3923220209724492, 0.6182892496624218},
        {1.4540698751743322, 0.626809421628518},
        {1.5511396204362184, 0.6488190768504346},
        {1.6355899386629067, 0.7014985139566801},
        {1.5583133646072271, 0.6380684875877585},
        {1.476708395875546, 0.5810078987344227},
        {1.545, 0.621}};

    mpp::TrajectoryFollower f;  // deployed-like defaults
    f.params.max_speed      = 0.5;
    f.params.goal_dist_tol  = 0.15;
    f.params.goal_ang_tol   = mrpt::DEG2RAD(12.0);
    f.params.arrival_radius = 0.3;
    f.setTrajectory(revTraj(pts, start.phi, goal.phi, 0.5));

    // Ackermann min turn radius ~0.36 m -> max curvature ~2.78 /m.
    const double kMaxCurv = 1.0 / 0.36;
    const auto   r        = simulateVerbose(f, start, pts, goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    const double hErr = mrpt::RAD2DEG(
        std::abs(mrpt::math::wrapToPi(r.finalPose.phi - goal.phi)));
    fprintf(
        stderr,
        "[result] reached=%d final=(%.3f,%.3f,%.1fdeg) dGoal=%.3f hErr=%.1fdeg "
        "steps=%d lastStatus=%d\n",
        r.reached, r.finalPose.x, r.finalPose.y, mrpt::RAD2DEG(r.finalPose.phi),
        dGoal, hErr, r.steps, static_cast<int>(r.lastStatus));

    // It must back cleanly into the standoff and terminate -- not turn away and
    // hang. Position is reached within tolerance; the terminal heading is
    // best-effort (an Ackermann robot cannot seat a differential-drive path's
    // in-place terminal rotation, and a downstream reactive controller
    // re-orients from the standoff in the real deployment), so only a loose
    // heading bound is asserted.
    EXPECT_TRUE(r.reached) << "follower failed to reach/terminate on a short "
                              "reverse goal (hung on the terminal cusp-loop)";
    EXPECT_LT(dGoal, 0.15);
    EXPECT_LT(hErr, 25.0);
}

// Regression: a second real capture of essentially the same short
// standoff-reverse maneuver as ShortReverseEntryLiveCase above (same start
// pose, same ~0.72 m goal, same small end-of-path wiggle) -- but this one
// carries the plan's *real* recorded per-point headings (the A* planner's
// waypoints from a live rosbag) instead of ShortReverseEntryLiveCase's
// tangent-reconstructed ones, and reproduces a live OffPathExceeded ~1.4 s
// after the follower picked up the plan.
TEST(TrajectoryFollower, ShortReverseEntryLiveCaseWithRecordedHeadings)
{
    struct XYYawDeg
    {
        double x, y, yaw_deg;
    };
    const std::vector<XYYawDeg> pts = {
        {1.0019, 0.5520, -152.84}, {1.0909, 0.5977, -152.84},
        {1.1799, 0.6433, -152.84}, {1.2689, 0.6890, -152.84},
        {1.3578, 0.7346, -152.84}, {1.4436, 0.7858, -145.46},
        {1.5222, 0.8476, -138.09}, {1.5921, 0.9190, -130.71},
        {1.6523, 0.9988, -123.33}, {1.7154, 1.0761, -135.13},
        {1.7926, 1.1387, -146.89}, {1.7274, 1.1555, -152.28}};

    mpp::Trajectory       tr;
    std::vector<TPoint2D> xy;
    for (const auto& p : pts)
    {
        tr.emplace_back(TPose2D(p.x, p.y, mrpt::DEG2RAD(p.yaw_deg)), 0.5);
        xy.emplace_back(p.x, p.y);
    }

    mpp::TrajectoryFollower f;  // deployed-like defaults
    f.params.max_speed      = 0.5;
    f.params.goal_dist_tol  = 0.15;
    f.params.goal_ang_tol   = mrpt::DEG2RAD(12.0);
    f.params.arrival_radius = 0.3;
    f.setTrajectory(tr);

    const TPose2D start(
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg));
    const TPose2D goal(
        pts.back().x, pts.back().y, mrpt::DEG2RAD(pts.back().yaw_deg));
    const double kMaxCurv = 1.0 / 0.36;  // Ackermann min turn radius ~0.36 m
    const auto   r        = simulateVerbose(f, start, xy, goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    fprintf(
        stderr,
        "[result] reached=%d final=(%.3f,%.3f,%.1fdeg) dGoal=%.3f "
        "steps=%d lastStatus=%d\n",
        r.reached, r.finalPose.x, r.finalPose.y, mrpt::RAD2DEG(r.finalPose.phi),
        dGoal, r.steps, static_cast<int>(r.lastStatus));

    EXPECT_NE(r.lastStatus, mpp::FollowerStatus::OffPathExceeded)
        << "follower lost track of the reverse path, exactly as on the real "
           "robot";
    EXPECT_TRUE(r.reached);
    EXPECT_LT(dGoal, 0.2);
}

// Reproduces the "rotates too aggressively" complaint from a real
// entry-alignment maneuver: a short, sharp turn where pure pursuit alone would
// ask for a tighter radius than the deployed (Ackermann) vehicle can make.
// min_turn_radius must clamp the *commanded* curvature directly, unlike
// max_lateral_accel (which only trades speed for curvature but never bounds
// it), so the follower never issues a physically infeasible turn.
TEST(TrajectoryFollower, ClampsCurvatureToMinTurnRadius)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {0.3, 0}, {0.3, 0.3}};
    mpp::TrajectoryFollower     f;
    f.params.max_speed       = 0.5;
    f.params.min_turn_radius = 0.36;  // a real Ackermann robot's turn limit
    f.setTrajectory(polyToTraj(pts, 0.5));

    TPose2D      robot   = {0, 0, 0};
    double       v       = 0;
    const double dt      = f.params.control_period;
    double       maxCurv = 0.0;
    for (int k = 0; k < 500; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        if (std::abs(tw.vx) > 1e-3)
            maxCurv = std::max(maxCurv, std::abs(tw.omega / tw.vx));
        robot = integrate(robot, tw.vx, tw.omega, dt);
        v     = tw.vx;
    }
    EXPECT_LE(maxCurv, 1.0 / f.params.min_turn_radius + 1e-6);
}

// A path with a cusp far from the final goal must slow toward that cusp too,
// not only toward the final goal: without this, the follower cruises at full
// speed right up to the direction reversal and then has to take the sharp
// just-past-cusp turn while still fast (the aggressive-looking behavior seen
// on a real deployment's short reverse-entry maneuver).
TEST(TrajectoryFollower, DeceleratesBeforeCusp)
{
    // (0,0) -> (5,0) -> (3,1): a cusp at s=5, well short of the final goal
    // distance check dominating (at x=4.9,y=0 the cusp is 0.1 m away in a
    // straight line, the goal 2.15 m away). The goal is off-axis so it never
    // physically coincides with a point on the outbound leg (unlike a
    // same-line "there and back" path, which would let the robot satisfy the
    // Euclidean goal-distance check early, without ever tracking arc-length
    // through the cusp).
    const std::vector<TPoint2D> pts = {{0, 0}, {5, 0}, {3, 1}};
    mpp::TrajectoryFollower     f;
    f.params.max_speed = 0.5;
    f.params.max_decel = 0.7;
    f.setTrajectory(polyToTraj(pts, 0.5));

    TPose2D      robot         = {0, 0, 0};
    double       v             = 0;
    const double dt            = f.params.control_period;
    double       speedNearCusp = -1;
    for (int k = 0; k < 1000; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        if (speedNearCusp < 0 && out.arc_length_s > 4.85 &&
            out.arc_length_s < 4.98)
            speedNearCusp = std::abs(tw.vx);
        robot = integrate(robot, tw.vx, tw.omega, dt);
        v     = tw.vx;
    }
    ASSERT_GE(speedNearCusp, 0.0) << "never sampled a point near the cusp";
    // Without decel-to-cusp this would still be at max_speed (0.5); within
    // 0.1-0.15 m of the cusp the cap is sqrt(2*0.7*d) = 0.37-0.46 m/s.
    EXPECT_LT(speedNearCusp, 0.48);
}

// --------------------------------------------------------------------------
// Regression: two REAL A* plans captured from a field-robot rosbag, both
// requested a few seconds apart by the same "approach then enter a narrow
// row" maneuver (an agricultural robot lining up with, then backing into, a
// crop row). Each waypoint's heading below is the *exact* orientation the
// live reference-path message carried for that pose (not a
// tangent-from-neighbor reconstruction), so the test reproduces precisely
// what a deployed `TrajectoryFollower`-based node consumed.
namespace
{
struct XYYawDeg
{
    double x, y, yaw_deg;
};

mpp::Trajectory mkTrajFromRecorded(
    const std::vector<XYYawDeg>& pts, double speed)
{
    mpp::Trajectory tr;
    for (const auto& p : pts)
    {
        tr.emplace_back(TPose2D(p.x, p.y, mrpt::DEG2RAD(p.yaw_deg)), speed);
    }
    return tr;
}

std::vector<TPoint2D> xyOf(const std::vector<XYYawDeg>& pts)
{
    std::vector<TPoint2D> out;
    for (const auto& p : pts) out.emplace_back(p.x, p.y);
    return out;
}

// Deployment-like tuning (small Ackermann field robot: ~0.36 m min turn
// radius), so this test exercises a realistic follower configuration.
void applyDeployedParams(mpp::TrajectoryFollower& f)
{
    f.params.max_speed         = 0.5;
    f.params.max_accel         = 0.5;
    f.params.max_decel         = 0.7;
    f.params.max_lateral_accel = 1.0;
    f.params.min_turn_radius   = 0.4;
    f.params.lookahead_max     = 1.5;
    f.params.lookahead_bend    = mrpt::DEG2RAD(25.0);
    f.params.goal_dist_tol     = 0.15;
    f.params.goal_ang_tol      = mrpt::DEG2RAD(12.0);
    f.params.max_cross_track   = 1.0;
    f.params.arrival_radius    = 0.3;
}
}  // namespace

TEST(TrajectoryFollower, ApproachStandoffLiveCase)
{
    const std::vector<XYYawDeg> pts = {
        {2.542, 2.945, -63.2},  {2.593, 2.858, -55.8},  {2.654, 2.780, -48.4},
        {2.725, 2.709, -41.0},  {2.805, 2.649, -33.6},  {2.886, 2.590, -38.1},
        {2.962, 2.526, -42.5},  {3.033, 2.455, -46.9},  {3.099, 2.380, -51.4},
        {3.155, 2.297, -60.2},  {3.192, 2.220, -67.8},  {3.215, 2.123, -86.2},
        {3.205, 2.024, -104.7}, {3.165, 1.933, -123.1}, {3.098, 1.859, -141.6},
        {3.178, 1.919, -144.5}, {3.260, 1.975, -147.4}, {3.236, 1.924, -152.4}};

    mpp::TrajectoryFollower f;
    applyDeployedParams(f);
    f.setTrajectory(mkTrajFromRecorded(pts, 0.5));

    const TPose2D start(
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg));
    const TPose2D goal(
        pts.back().x, pts.back().y, mrpt::DEG2RAD(pts.back().yaw_deg));
    const double kMaxCurv = 1.0 / 0.364;  // real Ackermann wheelbase limit
    const auto   r = simulateVerbose(f, start, xyOf(pts), goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(dGoal, 0.3);
    EXPECT_LT(r.maxCross, 0.5);
}

TEST(TrajectoryFollower, RowEntryCuspLiveCase)
{
    const std::vector<XYYawDeg> pts = {
        {3.047, 2.063, -17.8},  {3.146, 2.051, 4.4},    {3.242, 2.078, 26.5},
        {3.330, 2.125, 30.2},   {3.414, 2.178, 33.9},   {3.496, 2.236, 37.6},
        {3.573, 2.300, 41.3},   {3.646, 2.368, 44.9},   {3.714, 2.441, 48.6},
        {3.729, 2.458, 49.5},   {3.788, 2.538, 57.6},   {3.836, 2.626, 65.7},
        {3.859, 2.723, 87.8},   {3.843, 2.821, 110.0},  {3.792, 2.906, 132.1},
        {3.713, 2.966, 154.2},  {3.809, 2.941, 176.3},  {3.907, 2.953, -161.5},
        {3.994, 3.002, -139.4}, {4.056, 3.080, -117.3}, {4.090, 3.174, -103.2},
        {4.101, 3.273, -89.2},  {4.088, 3.372, -75.2},  {4.051, 3.464, -61.2},
        {4.102, 3.505, -62.4}};

    mpp::TrajectoryFollower f;
    applyDeployedParams(f);
    f.setTrajectory(mkTrajFromRecorded(pts, 0.5));

    const TPose2D start(
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg));
    const TPose2D goal(
        pts.back().x, pts.back().y, mrpt::DEG2RAD(pts.back().yaw_deg));
    const double kMaxCurv = 1.0 / 0.364;  // real Ackermann wheelbase limit
    const auto   r = simulateVerbose(f, start, xyOf(pts), goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    EXPECT_NE(r.lastStatus, mpp::FollowerStatus::OffPathExceeded)
        << "follower lost track of the cusp path, exactly as on the real robot";
    EXPECT_TRUE(r.reached);
    EXPECT_LT(dGoal, 0.3);
    EXPECT_LT(r.maxCross, 0.5);
}

TEST(TrajectoryFollower, ParamsYamlRoundTrip)
{
    mpp::TrajectoryFollower::Parameters p;
    p.max_speed                 = 0.33;
    p.lookahead_max             = 2.0;
    p.lookahead_bend            = mrpt::DEG2RAD(40.0);
    p.horizon                   = 2.5;
    p.stop_distance             = 0.42;
    p.slow_distance             = 1.75;
    p.min_turn_radius           = 0.4;
    p.max_curvature_rate        = 3.5;
    p.min_lookahead_arc         = 0.35;
    p.off_path_min_duration     = 1.25;
    p.anchor_time_constant      = 0.44;
    p.anchor_max_lin_rate       = 0.66;
    p.anchor_max_ang_rate       = 1.2;
    p.anchor_max_lin_divergence = 0.41;
    p.anchor_max_ang_divergence = 0.31;
    const auto y                = p.as_yaml();
    const auto p2 = mpp::TrajectoryFollower::Parameters::FromYAML(y);
    EXPECT_NEAR(p2.max_speed, 0.33, 1e-9);
    EXPECT_NEAR(p2.lookahead_max, 2.0, 1e-9);
    EXPECT_NEAR(p2.lookahead_bend, mrpt::DEG2RAD(40.0), 1e-9);
    EXPECT_NEAR(p2.horizon, 2.5, 1e-9);
    EXPECT_NEAR(p2.stop_distance, 0.42, 1e-9);
    EXPECT_NEAR(p2.slow_distance, 1.75, 1e-9);
    EXPECT_NEAR(p2.min_turn_radius, 0.4, 1e-9);
    EXPECT_NEAR(p2.max_curvature_rate, 3.5, 1e-9);
    EXPECT_NEAR(p2.min_lookahead_arc, 0.35, 1e-9);
    EXPECT_NEAR(p2.off_path_min_duration, 1.25, 1e-9);
    EXPECT_NEAR(p2.anchor_time_constant, 0.44, 1e-9);
    EXPECT_NEAR(p2.anchor_max_lin_rate, 0.66, 1e-9);
    EXPECT_NEAR(p2.anchor_max_ang_rate, 1.2, 1e-9);
    EXPECT_NEAR(p2.anchor_max_lin_divergence, 0.41, 1e-9);
    EXPECT_NEAR(p2.anchor_max_ang_divergence, 0.31, 1e-9);
}

// Short-term tracking runs on the smooth wheel odometry with the map->odom
// anchor slewed toward localization. A one-time relocalization jump in the
// localized pose must be absorbed gradually -- the commanded angular velocity
// must not lurch -- while the follower still reacts to the correction and
// reaches the goal. (Under a non-anchored, localization-only pursuit the same
// jump steps the tracked cross-track error and spikes the commanded omega.)
TEST(TrajectoryFollower, RelocalizationJumpDoesNotLurch)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {8, 0}};
    mpp::TrajectoryFollower     f;
    f.params.max_speed = 0.5;
    f.setTrajectory(polyToTraj(pts, 0.5));

    // "odom" is the smooth physical frame; map->odom is a fixed rotation +
    // offset, except for a one-time relocalization jump injected mid-drive.
    const TPose2D odomFromMap{1.5, -2.0, mrpt::DEG2RAD(30.0)};
    auto          toOdom = [&](const TPose2D& pm)
    {
        return (mrpt::poses::CPose2D(odomFromMap) + mrpt::poses::CPose2D(pm))
            .asTPose();
    };

    TPose2D mapPose{0, 0, 0};  // true robot pose (map frame)
    double  v         = 0;
    double  prevOmega = 0;
    double  maxDOmega = 0;
    double  maxAbsY   = 0;
    bool    reached   = false;
    for (int k = 0; k < 4000; k++)
    {
        // Localization reports the true map pose, except a one-time +0.25 m
        // lateral step from step 60 onward (a relocalization correction). The
        // wheel odometry stays continuous throughout.
        TPose2D locMap = mapPose;
        if (k >= 60) locMap.y += 0.25;

        const auto out = f.step(mkLoc(locMap), mkOdo(toOdom(mapPose), v));
        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            reached = true;
            break;
        }
        ASSERT_FALSE(out.command.points.empty());
        const auto tw = out.command.points.front().twist;
        if (k > 1)
            maxDOmega = std::max(maxDOmega, std::abs(tw.omega - prevOmega));
        prevOmega = tw.omega;
        mapPose = integrate(mapPose, tw.vx, tw.omega, f.params.control_period);
        v       = tw.vx;
        if (k > 60) maxAbsY = std::max(maxAbsY, std::abs(mapPose.y));
    }
    EXPECT_TRUE(reached);
    // The follower must actually act on the correction (not ignore it): the
    // true pose deviates toward the reported step, confirming omega was
    // genuinely exercised.
    EXPECT_GT(maxAbsY, 0.15)
        << "follower did not respond to the localization step";
    // ...but it is slewed in over many cycles, so the per-cycle change in
    // commanded angular velocity stays small even across the jump (measured
    // ~0.01 rad/s with the anchor slew vs ~0.11 for a localization-only pursuit
    // that steps the tracked pose in a single cycle).
    EXPECT_LT(maxDOmega, 0.05)
        << "commanded omega lurched on the relocalization jump";
}

// --------------------------------------------------------------------------
// Regression: two more REAL A* / generated-transition paths captured from a
// field-robot simulation rosbag, both ending in `OffPathExceeded` on the real
// deployment ("oscillates/dances and makes weird turns near the end of the
// path, even though the goal heading is already essentially reached").
namespace
{
// The real deployed follower-params.yaml tuning (not the looser test-only
// values `applyDeployedParams` above uses).
void applyIdmDeployedFollowerParams(mpp::TrajectoryFollower& f)
{
    f.params.max_speed         = 0.3;
    f.params.max_accel         = 0.3;
    f.params.max_decel         = 0.6;
    f.params.max_lateral_accel = 0.4;
    f.params.min_turn_radius   = 0.6;
    f.params.lookahead_max     = 1.8;
    f.params.lookahead_bend    = mrpt::DEG2RAD(25.0);
    f.params.goal_dist_tol     = 0.15;
    f.params.goal_ang_tol      = mrpt::DEG2RAD(12.0);
    f.params.max_cross_track   = 0.6;
    f.params.arrival_radius    = 0.3;
}
}  // namespace

// The last 3 knots of this real reverse-into-row path all carry the SAME
// (already-correct) goal heading, but their raw (x,y) has a small ~cm-scale
// direction wiggle (an artifact of the upstream path generator's terminal
// segment), which used to invert the discrete secant tangent right at the
// tail. On the real robot this shows up as the commanded omega slamming
// between +max and -max curvature (both directions saturate the
// min_turn_radius clamp) over the last ~1.5 m of an otherwise smooth arc,
// even though there is essentially no heading left to correct.
TEST(TrajectoryFollower, ReverseArcTerminalWiggleLiveCase)
{
    const std::vector<XYYawDeg> pts = {
        {0.9975, 0.4742, -153.71}, {1.3456, 0.6239, -153.71},
        {1.6849, 0.7915, -153.71}, {2.0241, 0.9591, -153.71},
        {2.3634, 1.1267, -153.71}, {2.7026, 1.2943, -153.71},
        {3.0419, 1.4619, -153.71}, {3.1881, 1.5465, -146.16},
        {3.3219, 1.6496, -138.62}, {3.4410, 1.7694, -131.07},
        {3.5434, 1.9037, -123.52}, {3.6272, 2.0504, -115.98},
        {3.6911, 2.2068, -108.43}, {3.7338, 2.3702, -100.88},
        {3.7547, 2.5379, -93.34},  {3.7534, 2.7068, -85.79},
        {3.7300, 2.8741, -78.24},  {3.6847, 3.0369, -70.70},
        {3.6185, 3.1923, -63.15},  {3.5912, 3.2905, -63.15},
        {3.5460, 3.3797, -63.15},  {3.5009, 3.4690, -63.15}};

    mpp::TrajectoryFollower f;
    applyIdmDeployedFollowerParams(f);
    f.setTrajectory(mkTrajFromRecorded(pts, 0.3));

    const TPose2D start(
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg));
    const TPose2D goal(
        pts.back().x, pts.back().y, mrpt::DEG2RAD(pts.back().yaw_deg));
    const double kMaxCurv = 1.0 / 0.364;  // real Ackermann wheelbase limit
    const auto   r = simulateVerbose(f, start, xyOf(pts), goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    EXPECT_NE(r.lastStatus, mpp::FollowerStatus::OffPathExceeded)
        << "follower lost track of the reverse arc, exactly as on the real "
           "robot";
    EXPECT_TRUE(r.reached);
    EXPECT_LT(dGoal, 0.3);
    EXPECT_LT(r.maxCross, 0.5);
}

// A longer, continuously-curving ~155 deg turn (no terminal wiggle -- the
// knots are smooth throughout). On the real robot this path diverges instead
// of oscillating: once the along-path projection pins near the goal while the
// vehicle is still well off in heading/cross-track, the pure-pursuit target
// ends up geometrically inside the vehicle's own (clamped) minimum turning
// circle, so commanding max curvature cannot reduce the error -- the vehicle
// keeps circling at cruise speed and the error grows until it collides /
// trips OffPathExceeded, instead of slowing down to make the turn.
TEST(TrajectoryFollower, SharpCurveSpiralDivergenceLiveCase)
{
    const std::vector<XYYawDeg> pts = {
        {3.5917, 3.6833, -65.71},  {3.6717, 3.4701, -73.19},
        {3.7231, 3.2483, -80.69},  {3.7452, 3.0217, -88.19},
        {3.7375, 2.7941, -95.69},  {3.7001, 2.5695, -103.19},
        {3.6337, 2.3516, -110.69}, {3.5395, 2.1443, -118.19},
        {3.4190, 1.9511, -125.69}, {3.2743, 1.7753, -133.19},
        {3.1080, 1.6198, -140.69}, {2.9227, 1.4874, -148.19},
        {2.7218, 1.3803, -155.69}, {2.6389, 1.3209, -155.69},
        {2.5477, 1.2797, -155.69}, {2.4566, 1.2386, -155.69}};

    mpp::TrajectoryFollower f;
    applyIdmDeployedFollowerParams(f);
    f.setTrajectory(mkTrajFromRecorded(pts, 0.3));

    const TPose2D start(
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg));
    const TPose2D goal(
        pts.back().x, pts.back().y, mrpt::DEG2RAD(pts.back().yaw_deg));
    const double kMaxCurv = 1.0 / 0.364;  // real Ackermann wheelbase limit
    const auto   r = simulateVerbose(f, start, xyOf(pts), goal, kMaxCurv);

    const double dGoal =
        std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    EXPECT_NE(r.lastStatus, mpp::FollowerStatus::OffPathExceeded)
        << "follower spiraled off the sharp curve, exactly as on the real "
           "robot";
    EXPECT_TRUE(r.reached);
    EXPECT_LT(dGoal, 0.3);
    EXPECT_LT(r.maxCross, 0.5);
}

// --------------------------------------------------------------------------
// Regression: `max_omega_rate` must actually do what it documents -- bound
// the *commanded* omega's own step size between control cycles -- on a real
// captured path (the same sharp live-case curve above), not just in a toy
// scenario. This is a mechanism-level check (the rate limiter is trivially
// self-verifying), independent of any particular vehicle/actuator model.
TEST(TrajectoryFollower, MaxOmegaRateLimitsCommandedOmegaStep)
{
    const std::vector<XYYawDeg> pts = {
        {3.5917, 3.6833, -65.71},  {3.6717, 3.4701, -73.19},
        {3.7231, 3.2483, -80.69},  {3.7452, 3.0217, -88.19},
        {3.7375, 2.7941, -95.69},  {3.7001, 2.5695, -103.19},
        {3.6337, 2.3516, -110.69}, {3.5395, 2.1443, -118.19},
        {3.4190, 1.9511, -125.69}, {3.2743, 1.7753, -133.19},
        {3.1080, 1.6198, -140.69}, {2.9227, 1.4874, -148.19},
        {2.7218, 1.3803, -155.69}, {2.6389, 1.3209, -155.69},
        {2.5477, 1.2797, -155.69}, {2.4566, 1.2386, -155.69}};

    mpp::TrajectoryFollower f;
    applyIdmDeployedFollowerParams(f);
    constexpr double kMaxOmegaRate = 3.0;  // [rad/s^2]
    f.params.max_omega_rate        = kMaxOmegaRate;
    f.setTrajectory(mkTrajFromRecorded(pts, 0.3));

    TPose2D robot = {
        pts.front().x, pts.front().y, mrpt::DEG2RAD(pts.front().yaw_deg)};
    double       v           = 0;
    double       prevOmega   = 0;
    double       maxDOmega   = 0;
    const double dt          = f.params.control_period;
    const double allowedStep = kMaxOmegaRate * dt + 1e-6;
    for (int k = 0; k < 2000; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        if (k > 0)
        {
            maxDOmega = std::max(maxDOmega, std::abs(tw.omega - prevOmega));
        }
        prevOmega = tw.omega;
        robot     = integrate(robot, tw.vx, tw.omega, dt);
        v         = tw.vx;
    }
    EXPECT_LE(maxDOmega, allowedStep)
        << "commanded omega stepped by more than max_omega_rate allows";
}

// --------------------------------------------------------------------------
// Regression: `min_lookahead_dist` must keep the pure-pursuit target's
// Euclidean distance from the vehicle from collapsing as the vehicle closes
// in on a goal that is not perfectly centered/aligned -- the regime where
// curvature (~ yr/Ld^2) becomes highly sensitive to small position noise
// (even though min_turn_radius already bounds the *magnitude* of the
// resulting curvature either way, a collapsed Ld means a small Cartesian
// perturbation swings the *bearing* to the target -- and hence the sign of
// the commanded turn -- far more than the same perturbation would at a
// healthy lookahead distance; that bearing sensitivity, not the clamped
// magnitude, is what drives cycle-to-cycle oscillation). A short,
// nearly-finished path with the vehicle sitting close to the end but with a
// real lateral offset (not yet within goal tolerance) isolates the mechanism
// without depending on any specific captured trajectory.
TEST(TrajectoryFollower, MinLookaheadDistBoundsLookaheadDistance)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {2, 0}};
    const TPose2D               robotPose{1.95, 0.3, 0};

    double LdNoFloor = 0;
    {
        mpp::TrajectoryFollower f;
        f.params.min_turn_radius    = 0.6;
        f.params.min_lookahead_dist = 0.0;  // disabled
        f.setTrajectory(polyToTraj(pts, 0.3));
        const auto out = f.step(mkLoc(robotPose), mkOdo(robotPose, 0.3));
        LdNoFloor      = std::hypot(
                 out.lookahead_point.x - robotPose.x,
                 out.lookahead_point.y - robotPose.y);
    }

    double LdWithFloor = 0;
    {
        mpp::TrajectoryFollower f;
        f.params.min_turn_radius    = 0.6;
        f.params.min_lookahead_dist = 0.5;
        f.setTrajectory(polyToTraj(pts, 0.3));
        const auto out = f.step(mkLoc(robotPose), mkOdo(robotPose, 0.3));
        LdWithFloor    = std::hypot(
               out.lookahead_point.x - robotPose.x,
               out.lookahead_point.y - robotPose.y);
    }

    constexpr double kMinTurnRadius = 0.6;
    EXPECT_LT(LdNoFloor, kMinTurnRadius)
        << "test setup should reach the collapsed-Ld regime without the floor";
    EXPECT_GE(LdWithFloor, kMinTurnRadius - 1e-6)
        << "the floor should keep the lookahead at least min_turn_radius away";
    EXPECT_GT(LdWithFloor, LdNoFloor);
}

// --------------------------------------------------------------------------
// Accuracy / noise-robustness tests, driven by the analysis of a real
// wavy/zig-zagging deployment: the localization feed carried 10-35 cm of
// motion-correlated jitter, the bend-capped lookahead collapsed the pursuit
// gain window on the (arc-shaped) row-transition paths, and the omega-domain
// rate limiter turned the resulting saturation into a slow limit-cycle
// weave. These tests pin the fixes: the everywhere-enforced Euclidean
// lookahead floor, the curvature-domain rate limit, the anchor low-pass +
// divergence bound, and the off-path debounce.
namespace
{
// Tuning mirroring the post-analysis deployed follower-params.yaml.
void applyAccuracyTunedParams(mpp::TrajectoryFollower& f)
{
    f.params.max_speed                 = 0.3;
    f.params.max_accel                 = 0.2;
    f.params.max_decel                 = 0.6;
    f.params.max_lateral_accel         = 0.2;
    f.params.min_turn_radius           = 0.5;
    f.params.max_omega_rate            = 0.0;  // superseded by curvature rate
    f.params.max_curvature_rate        = 3.0;
    f.params.lookahead_max             = 1.8;
    f.params.lookahead_bend            = mrpt::DEG2RAD(25.0);
    f.params.min_lookahead_dist        = 0.8;
    f.params.goal_dist_tol             = 0.25;
    f.params.goal_ang_tol              = mrpt::DEG2RAD(35.0);
    f.params.max_cross_track           = 0.6;
    f.params.off_path_min_duration     = 1.0;
    f.params.arrival_radius            = 0.45;
    f.params.control_period            = 0.05;
    f.params.anchor_time_constant      = 0.4;
    f.params.anchor_max_lin_divergence = 0.4;
    f.params.anchor_max_ang_divergence = 0.4;
}

// The row-transition shape the deployed robot drives between crop rows:
// a straight lead-in, a 90 deg arc of R = 1.8 m, and a straight lead-out,
// sampled at ~0.25 m knots (the same knot pitch as the real generated
// paths).
std::vector<TPoint2D> rowTransitionArcPts()
{
    std::vector<TPoint2D> pts;
    const double          R = 1.8;
    for (double x = 0.0; x < 2.0; x += 0.25) pts.push_back({x, 0.0});
    // Arc center at (2, R): from heading 0 to heading +90 deg.
    for (double a = 0.0; a <= M_PI / 2 + 1e-9; a += 0.25 / R)
        pts.push_back({2.0 + R * std::sin(a), R - R * std::cos(a)});
    for (double y = R + 0.25; y < R + 1.25; y += 0.25)
        pts.push_back({2.0 + R, y});
    return pts;
}
}  // namespace

// With a perfect localization, the accuracy-tuned parameters must track the
// deployed row-transition arc tightly. Pure pursuit commands exactly the
// arc's curvature once on it (steady-state arc error ~0); the residual is
// the anticipatory turn-in transient at the reference's curvature *step*
// (straight -> arc, ~9 cm here), which is inherent to lookahead-based
// tracking of a curvature-discontinuous polyline: a finite-steering-slew
// vehicle must begin steering before the step either way. The budget below
// bounds that transient and, mainly, guards against any regression of the
// Euclidean lookahead floor / curvature rate limiter re-widening it.
TEST(TrajectoryFollower, ArcTrackingAccuracyOnRowTransition)
{
    const auto              pts = rowTransitionArcPts();
    mpp::TrajectoryFollower f;
    applyAccuracyTunedParams(f);
    f.setTrajectory(polyToTraj(pts, 0.3));

    const auto r = simulate(f, {0, 0, 0}, pts, 8000);
    EXPECT_TRUE(r.reached);
    EXPECT_LT(r.maxCross, 0.10)
        << "corner-cutting/overshoot on the reference arc exceeds the "
           "accuracy budget";
}

// The curvature-domain rate limiter must bound the per-cycle change of the
// commanded curvature (omega/v), the quantity a steering actuator actually
// tracks -- including across speed-profile changes, where an omega-domain
// limit would corrupt the executed radius instead.
TEST(TrajectoryFollower, CurvatureRateLimitBoundsCommandedCurvatureStep)
{
    const auto              pts = rowTransitionArcPts();
    mpp::TrajectoryFollower f;
    applyAccuracyTunedParams(f);
    constexpr double kRate      = 3.0;  // [1/m per s]
    f.params.max_curvature_rate = kRate;
    f.setTrajectory(polyToTraj(pts, 0.3));

    TPose2D      robot       = {0, 0, 0};
    double       v           = 0;
    double       prevCurv    = 0;
    bool         havePrev    = false;
    double       maxDCurv    = 0;
    const double dt          = f.params.control_period;
    const double allowedStep = kRate * dt + 1e-6;
    for (int k = 0; k < 4000; k++)
    {
        const auto out = f.step(mkLoc(robot), mkOdo(robot, v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;
        if (std::abs(tw.vx) > 1e-3)
        {
            const double curv = tw.omega / tw.vx;
            if (havePrev)
            {
                maxDCurv = std::max(maxDCurv, std::abs(curv - prevCurv));
            }
            prevCurv = curv;
            havePrev = true;
        }
        else
        {
            // Near-zero speed: curvature is not observable from omega/v, and
            // across a run of such cycles the limiter may legitimately step
            // it by several allowed increments -- only compare consecutive
            // driving cycles.
            havePrev = false;
        }
        robot = integrate(robot, tw.vx, tw.omega, dt);
        v     = tw.vx;
    }
    ASSERT_TRUE(havePrev);
    EXPECT_LE(maxDCurv, allowedStep)
        << "commanded curvature stepped faster than max_curvature_rate";
}

// Closed loop with a *jittery* localization (deterministic multi-sine noise
// with the amplitude/frequency content measured on the real system) and a
// smooth odometry: the anchor low-pass + divergence bound + lookahead floor
// + curvature rate limit together must keep the TRUE trajectory close to
// the reference, with no limit-cycle weave (bounded omega sign flips).
TEST(TrajectoryFollower, NoisyLocalizationDoesNotLimitCycle)
{
    const auto              pts = rowTransitionArcPts();
    mpp::TrajectoryFollower f;
    applyAccuracyTunedParams(f);
    f.setTrajectory(polyToTraj(pts, 0.3));

    // odom frame: fixed offset from map (smooth wheel odometry).
    const TPose2D odomFromMap{2.0, -1.0, mrpt::DEG2RAD(20.0)};
    auto          toOdom = [&](const TPose2D& pm)
    {
        return (mrpt::poses::CPose2D(odomFromMap) + mrpt::poses::CPose2D(pm))
            .asTPose();
    };

    TPose2D      mapPose{0, 0, 0};  // true robot pose
    double       v         = 0;
    const double dt        = f.params.control_period;
    double       t         = 0;
    double       maxCross  = 0;
    double       sumSqE    = 0;
    int          nE        = 0;
    int          omegaFlips = 0;
    int          lastSign   = 0;
    bool         reached    = false;
    for (int k = 0; k < 12000; k++)
    {
        t += dt;
        // Deterministic localization jitter: ~10 cm lateral-ish + ~3 deg yaw
        // at sub-Hz frequencies (the measured character of the real feed).
        TPose2D locMap = mapPose;
        locMap.x += 0.07 * std::sin(2 * M_PI * 0.45 * t + 0.3) +
                    0.03 * std::sin(2 * M_PI * 1.3 * t);
        locMap.y += 0.10 * std::sin(2 * M_PI * 0.55 * t) +
                    0.04 * std::sin(2 * M_PI * 1.7 * t + 1.1);
        locMap.phi += mrpt::DEG2RAD(3.0) * std::sin(2 * M_PI * 0.6 * t + 0.5);

        const auto out = f.step(mkLoc(locMap), mkOdo(toOdom(mapPose), v));
        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            reached = true;
            break;
        }
        if (out.command.points.empty()) break;
        const auto tw = out.command.points.front().twist;

        // Omega sign flips (with hysteresis) = the limit-cycle signature.
        const int sgn = tw.omega > 0.03 ? 1 : (tw.omega < -0.03 ? -1 : 0);
        if (sgn != 0 && lastSign != 0 && sgn != lastSign) omegaFlips++;
        if (sgn != 0) lastSign = sgn;

        mapPose = integrate(mapPose, tw.vx, tw.omega, dt);
        v       = tw.vx;

        const double e = distToPolyline(pts, {mapPose.x, mapPose.y});
        maxCross       = std::max(maxCross, e);
        sumSqE += e * e;
        nE++;
    }
    const double rmsE = std::sqrt(sumSqE / std::max(1, nE));
    EXPECT_TRUE(reached);
    EXPECT_LT(rmsE, 0.10) << "true-pose cross-track RMS too large under "
                             "localization jitter";
    EXPECT_LT(maxCross, 0.30);
    // A limit cycle flips the commanded turn direction every couple of
    // seconds for the whole run; tracking the (one-bend) reference needs
    // only a handful of sign changes.
    EXPECT_LT(omegaFlips, 12)
        << "commanded omega oscillates (limit-cycle weave)";
}

// The anchor complementary filter must *attenuate* localization jitter, not
// amplify it: the control-pose cross-track the follower steers on must be
// smoother than the raw injected jitter (on the real system the un-filtered
// anchor slew made the control pose 2x WORSE than the raw localization).
TEST(TrajectoryFollower, AnchorFilterAttenuatesLocalizationJitter)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {12, 0}};
    mpp::TrajectoryFollower     f;
    applyAccuracyTunedParams(f);
    f.setTrajectory(polyToTraj(pts, 0.3));

    const TPose2D odomFromMap{1.0, 2.0, mrpt::DEG2RAD(-15.0)};
    auto          toOdom = [&](const TPose2D& pm)
    {
        return (mrpt::poses::CPose2D(odomFromMap) + mrpt::poses::CPose2D(pm))
            .asTPose();
    };

    TPose2D          mapPose{0, 0, 0};
    double           v          = 0;
    const double     dt         = f.params.control_period;
    double           t          = 0;
    constexpr double kJitterAmp = 0.15;  // [m] pure lateral sine
    double           sumSqCtrl  = 0;
    int              n          = 0;
    double           maxTrueE   = 0;
    for (int k = 0; k < 6000; k++)
    {
        t += dt;
        TPose2D locMap = mapPose;
        locMap.y += kJitterAmp * std::sin(2 * M_PI * 0.7 * t);

        const auto out = f.step(mkLoc(locMap), mkOdo(toOdom(mapPose), v));
        if (out.status == mpp::FollowerStatus::ReachedGoal) break;
        if (out.command.points.empty()) break;

        // Steady state only (skip the initial anchor convergence).
        if (k > 100)
        {
            sumSqCtrl += out.cross_track_err * out.cross_track_err;
            n++;
        }
        const auto tw = out.command.points.front().twist;
        mapPose       = integrate(mapPose, tw.vx, tw.omega, dt);
        v             = tw.vx;
        maxTrueE      = std::max(
                 maxTrueE, distToPolyline(pts, {mapPose.x, mapPose.y}));
    }
    ASSERT_GT(n, 500);
    const double ctrlStd   = std::sqrt(sumSqCtrl / n);
    const double jitterStd = kJitterAmp / std::sqrt(2.0);
    EXPECT_LT(ctrlStd, 0.6 * jitterStd)
        << "control-pose cross-track (std " << ctrlStd
        << " m) is not attenuated vs the raw jitter (std " << jitterStd
        << " m)";
    EXPECT_LT(maxTrueE, 0.20)
        << "true trajectory deviates too much under pure localization jitter";
}

// The correction filter must behave the same whether the robot is 2 m or
// 300 m from where its odometry started. An earlier formulation filtered the
// map->odom anchor, whose partially-applied yaw corrections rotate the
// composed control pose about the odom ORIGIN: localization yaw jitter swung
// the implied anchor position by (jitter x distance-from-origin), saturating
// the filter and pinning the control pose at the divergence bound -- so
// tracking was clean on the first rows of a field mission and degraded
// linearly with distance driven. Filtered at the vehicle, yaw jitter is just
// local heading noise: the true driven path must stay as clean far from the
// odom origin as near it.
TEST(TrajectoryFollower, YawJitterFarFromOdomOriginDoesNotDegrade)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {12, 0}};

    // Drives the same straight path under localization yaw jitter; only the
    // odom-origin offset differs between the two cases.
    auto runCase = [&](const TPose2D& odomFromMap) -> double
    {
        mpp::TrajectoryFollower f;
        applyAccuracyTunedParams(f);
        f.setTrajectory(polyToTraj(pts, 0.3));

        auto toOdom = [&](const TPose2D& pm)
        {
            return (mrpt::poses::CPose2D(odomFromMap) +
                    mrpt::poses::CPose2D(pm))
                .asTPose();
        };

        TPose2D          mapPose{0, 0, 0};
        double           v  = 0;
        const double     dt = f.params.control_period;
        double           t  = 0;
        constexpr double kYawJitterAmp = mrpt::DEG2RAD(2.0);
        double           maxTrueE      = 0;
        bool             reached       = false;
        for (int k = 0; k < 6000; k++)
        {
            t += dt;
            TPose2D locMap = mapPose;
            locMap.phi += kYawJitterAmp * std::sin(2 * M_PI * 0.7 * t);

            const auto out = f.step(mkLoc(locMap), mkOdo(toOdom(mapPose), v));
            if (out.status == mpp::FollowerStatus::ReachedGoal)
            {
                reached = true;
                break;
            }
            if (out.command.points.empty())
            {
                break;
            }
            const auto tw = out.command.points.front().twist;
            mapPose       = integrate(mapPose, tw.vx, tw.omega, dt);
            v             = tw.vx;
            // Steady state only (skip the initial convergence).
            if (k > 100)
            {
                maxTrueE = std::max(
                    maxTrueE, distToPolyline(pts, {mapPose.x, mapPose.y}));
            }
        }
        EXPECT_TRUE(reached);
        return maxTrueE;
    };

    const double eNear = runCase({1.0, 2.0, mrpt::DEG2RAD(-15.0)});
    const double eFar  = runCase({250.0, -180.0, mrpt::DEG2RAD(30.0)});

    EXPECT_LT(eNear, 0.10) << "yaw jitter degrades tracking near the origin";
    // The invariant is distance-independence: the far case must track as
    // cleanly as the near one (the anchor-based formulation measured ~23x
    // worse here), up to a small numeric epsilon.
    EXPECT_LT(eFar, 2.0 * eNear + 0.005)
        << "tracking error grows with the distance from the odom origin "
           "(lever-arm-amplified correction): eNear="
        << eNear << " m, eFar=" << eFar << " m";
}

// Approaching a cusp, the lookahead arc-length window collapses (it may not
// cross the cusp), which used to collapse the *Euclidean* lookahead distance
// with it -- full-authority steering on centimeter errors right where the
// vehicle is slowing to reverse. The floor must hold there too, by
// extrapolating along the incoming tangent past the cusp.
TEST(TrajectoryFollower, CuspApproachKeepsLookaheadDistance)
{
    // Forward to (2,0), cusp, then back-and-left to (1,1).
    const std::vector<TPoint2D> pts = {
        {0, 0}, {0.5, 0}, {1.0, 0}, {1.5, 0}, {2.0, 0},
        {1.8, 0.2}, {1.6, 0.4}, {1.4, 0.6}, {1.2, 0.8}, {1.0, 1.0}};
    mpp::TrajectoryFollower f;
    applyAccuracyTunedParams(f);
    f.setTrajectory(polyToTraj(pts, 0.3));

    // Robot on-path, 0.15 m short of the cusp.
    const TPose2D robot{1.85, 0.0, 0.0};
    const auto    out = f.step(mkLoc(robot), mkOdo(robot, 0.2));
    const double  Ld  = std::hypot(
         out.lookahead_point.x - robot.x, out.lookahead_point.y - robot.y);
    const double minLd =
        std::max(f.params.min_lookahead_dist, f.params.min_turn_radius);
    EXPECT_GE(Ld, minLd - 1e-6)
        << "lookahead distance collapsed approaching the cusp";
}

// A transient localization spike must not abort the mission: with
// off_path_min_duration set, OffPathExceeded fires only after the
// cross-track has stayed over the limit continuously for that long.
TEST(TrajectoryFollower, OffPathDebounceSuppressesTransientSpike)
{
    const std::vector<TPoint2D> pts = {{0, 0}, {8, 0}};
    mpp::TrajectoryFollower     f;
    applyAccuracyTunedParams(f);
    f.params.max_cross_track       = 0.6;
    f.params.off_path_min_duration = 1.0;
    f.setTrajectory(polyToTraj(pts, 0.3));

    const auto t0 = mrpt::Clock::now();

    // A 2 m spike: immediately over the limit, but too fresh to latch.
    auto locSpike      = mkLoc({1.0, 2.0, 0.0});
    locSpike.timestamp = t0;
    auto out = f.step(locSpike, mkOdo({1.0, 2.0, 0.0}, 0.3));
    EXPECT_NE(out.status, mpp::FollowerStatus::OffPathExceeded)
        << "a single-cycle spike must not trip the fault";

    // Back on the path 0.3 s later: the debounce window resets.
    auto locOk      = mkLoc({1.1, 0.0, 0.0});
    locOk.timestamp = t0 + std::chrono::milliseconds(300);
    out             = f.step(locOk, mkOdo({1.1, 0.0, 0.0}, 0.3));
    EXPECT_EQ(out.status, mpp::FollowerStatus::Running);

    // Sustained deviation: over the limit continuously for > 1 s -> fault.
    for (int i = 0; i <= 12; i++)
    {
        auto locOff      = mkLoc({1.2, 2.0, 0.0});
        locOff.timestamp = t0 + std::chrono::milliseconds(400 + i * 100);
        out              = f.step(locOff, mkOdo({1.2, 2.0, 0.0}, 0.3));
    }
    EXPECT_EQ(out.status, mpp::FollowerStatus::OffPathExceeded)
        << "a sustained deviation must still trip the fault";
}
