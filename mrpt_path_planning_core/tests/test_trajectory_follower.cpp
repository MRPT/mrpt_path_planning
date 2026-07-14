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
    const auto    gateOut =
        f.step(mkLoc(onForwardLeg), mkOdo(onForwardLeg, 0.3));
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
    f.params.lookahead_min     = 0.4;
    f.params.lookahead_max     = 1.5;
    f.params.lookahead_time    = 1.0;
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
    p.max_speed       = 0.33;
    p.lookahead_max   = 2.0;
    p.horizon         = 2.5;
    p.stop_distance   = 0.42;
    p.slow_distance   = 1.75;
    p.min_turn_radius = 0.4;
    const auto y      = p.as_yaml();
    const auto p2     = mpp::TrajectoryFollower::Parameters::FromYAML(y);
    EXPECT_NEAR(p2.max_speed, 0.33, 1e-9);
    EXPECT_NEAR(p2.lookahead_max, 2.0, 1e-9);
    EXPECT_NEAR(p2.horizon, 2.5, 1e-9);
    EXPECT_NEAR(p2.stop_distance, 0.42, 1e-9);
    EXPECT_NEAR(p2.slow_distance, 1.75, 1e-9);
    EXPECT_NEAR(p2.min_turn_radius, 0.4, 1e-9);
}
