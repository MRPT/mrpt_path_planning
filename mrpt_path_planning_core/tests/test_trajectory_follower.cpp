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

// --------------------------------------------------------------------------
// Regression: a short reverse entry leg (reproduces the live NAVIGATE_ZONE
// "weird turn, could not finish" on the real robot). The planner backs the
// robot ~0.55 m into a row mouth: start and goal body headings are nearly
// aligned (~ -153 deg), so it should be an almost-straight reverse. The final
// reference pose carries the GOAL BODY heading (opposite the travel direction,
// as the real A* waypoints do), not the last-segment travel direction.
namespace
{
// Build a reverse trajectory: polyline points + an explicit final body heading
// (the goal orientation the robot must arrive at, tail-first).
mpp::Trajectory revTraj(
    const std::vector<TPoint2D>& pts, double goalHeading, double speed)
{
    mpp::Trajectory tr = polyToTraj(pts, speed);
    if (!tr.empty()) tr.back().pose.phi = goalHeading;
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
                mrpt::RAD2DEG(out.heading_err),
                static_cast<int>(out.status));

        if (out.status == mpp::FollowerStatus::ReachedGoal)
        {
            r.reached = true;
            break;
        }
        if (out.command.points.empty()) break;
        r.maxSpeed = std::max(r.maxSpeed, vx);
        // Ackermann steering limit: curvature |omega/v| capped at maxCurv, so an
        // in-place spin (omega large, v ~ 0) is physically impossible (a car-like
        // robot must roll to turn). maxCurv <= 0 => ideal unicycle.
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
    // Captured from the real robot (map frame): start pose and the entry
    // standoff goal for block 59, a ~0.55 m reverse with aligned headings.
    const TPose2D start(1.001, 0.553, mrpt::DEG2RAD(-152.7));
    const TPose2D goal(1.545, 0.621, mrpt::DEG2RAD(-153.0));

    // The REAL 11 A* waypoints captured live (make_plan_from_to, obstacle-free).
    // Note the cusps: the positions overshoot to (1.636,0.701) then reverse back
    // to (1.477,0.581) before the goal -- a differential-drive maneuver with
    // direction reversals, not a straight reverse.
    const std::vector<TPoint2D> pts = {
        {1.001, 0.553},          {1.0946131281188594, 0.5874501963198541},
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
    f.setTrajectory(revTraj(pts, goal.phi, 0.5));

    // Ackermann min turn radius ~0.36 m (tutabot) -> max curvature ~2.78 /m.
    const double kMaxCurv = 1.0 / 0.36;
    const auto   r        = simulateVerbose(f, start, pts, goal, kMaxCurv);

    const double dGoal = std::hypot(r.finalPose.x - goal.x, r.finalPose.y - goal.y);
    const double hErr  = mrpt::RAD2DEG(
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
    // in-place terminal rotation, and the downstream corridor-follower
    // re-orients from the standoff), so only a loose heading bound is asserted.
    EXPECT_TRUE(r.reached) << "follower failed to reach/terminate on a short "
                              "reverse goal (hung on the terminal cusp-loop)";
    EXPECT_LT(dGoal, 0.15);
    EXPECT_LT(hErr, 25.0);
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
