/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * Closed-loop path-following accuracy of TrajectoryFollower under realistic
 * sensing and actuation, judged against the simulated ground truth.
 *
 * Unlike test_trajectory_follower.cpp (perfect, instantaneous pose), the
 * simulated stack here mimics a real robot:
 *  - Wheel odometry with scale errors (so map->odom drifts), published at a
 *    high rate with a small delay.
 *  - Localization published at a lower rate, arriving with a latency
 *    (processing time) and carrying the timestamp of the sensor data it was
 *    computed from, plus noise.
 *  - Command transport delay, an optional first-order velocity lag of the
 *    platform, and control period jitter.
 *
 * Reference paths are both hand-built (arcs, cusps, short maneuvers next to
 * the goal) and planned by TPS_Astar with differential-drive C-PTGs.
 *
 * Every case checks, on the ground truth: the maximum distance to the
 * reference path, the final position and heading errors, and that the goal
 * is not reported as reached while the robot is actually out of tolerance.
 *
 * Set MPP_FOLLOWER_TRACE_DIR to dump each run (reference path and ground
 * truth) as CSV files for plotting.
 */

#include <gtest/gtest.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/TrajectoryFollower.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/data/PlannerInput.h>
#include <mpp/interfaces/ObstacleSource.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>

#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace
{
using mrpt::math::TPoint2D;
using mrpt::math::TPose2D;

// ---------------------------------------------------------------------------
// Simulated sensing / actuation conditions
// ---------------------------------------------------------------------------
struct StackConditions
{
    std::string name;

    double loc_period    = 0.0;  //!< [s] 0: a new pose every control cycle
    double loc_latency   = 0.0;  //!< [s] age of a pose when it is delivered
    double loc_noise_xy  = 0.0;  //!< [m] std. dev.
    double loc_noise_phi = 0.0;  //!< [rad] std. dev.
    /** If false, poses are stamped with the delivery time instead of the
     * sensor time (a node that cannot report the true pose age). */
    bool loc_true_stamps = true;

    double odo_latency     = 0.0;  //!< [s]
    double odo_scale_error = 0.0;  //!< relative error of odometry increments

    double cmd_latency   = 0.0;  //!< [s] command transport delay
    double plant_tau     = 0.0;  //!< [s] velocity lag (0: ideal twist)
    double period_jitter = 0.0;  //!< [s] uniform +/- on the control period
};

StackConditions idealStack()
{
    StackConditions c;
    c.name = "ideal";
    return c;
}

// A typical particle filter localization: 10 Hz, 0.3 s of processing, a few
// cm of noise; odometry with 2-3% scale errors; a small command delay.
StackConditions typicalStack()
{
    StackConditions c;
    c.name            = "typical";
    c.loc_period      = 0.1;
    c.loc_latency     = 0.3;
    c.loc_noise_xy    = 0.02;
    c.loc_noise_phi   = mrpt::DEG2RAD(0.5);
    c.odo_latency     = 0.02;
    c.odo_scale_error = 0.03;
    c.cmd_latency     = 0.05;
    c.period_jitter   = 0.01;
    return c;
}

// An overloaded computer: slow, late localization and an irregular loop.
StackConditions overloadedStack()
{
    StackConditions c = typicalStack();
    c.name            = "overloaded";
    c.loc_period      = 0.25;
    c.loc_latency     = 1.0;
    c.odo_latency     = 0.1;
    c.cmd_latency     = 0.1;
    c.period_jitter   = 0.03;
    return c;
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
TPose2D integrate(const TPose2D& p, double v, double w, double dt)
{
    double x   = p.x;
    double y   = p.y;
    double phi = p.phi;
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

double distToPolyline(const mpp::Trajectory& tr, const TPoint2D& q)
{
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < tr.size(); i++)
    {
        const auto&  a    = tr[i].pose;
        const auto&  b    = tr[i + 1].pose;
        const double sx   = b.x - a.x;
        const double sy   = b.y - a.y;
        const double len2 = sx * sx + sy * sy;
        double       t =
            len2 > 1e-12 ? ((q.x - a.x) * sx + (q.y - a.y) * sy) / len2 : 0.0;
        t               = std::clamp(t, 0.0, 1.0);
        const double px = a.x + t * sx;
        const double py = a.y + t * sy;
        best            = std::min(best, std::hypot(q.x - px, q.y - py));
    }
    return best;
}

struct TimedPose
{
    double  t = 0;
    TPose2D pose;
};

// Ground truth sample, with the follower state at that time:
struct TraceSample
{
    double  t = 0;
    TPose2D pose;
    double  v      = 0;  //!< actual speed [m/s]
    double  s      = 0;  //!< follower arc-length progress [m]
    int     status = 0;  //!< follower status
};

// Pose at time `t` from a time-ordered history (nearest earlier sample).
TPose2D poseAt(const std::deque<TimedPose>& h, double t)
{
    for (auto it = h.rbegin(); it != h.rend(); ++it)
    {
        if (it->t <= t + 1e-9) { return it->pose; }
    }
    return h.front().pose;
}

struct RunResult
{
    bool                     ended       = false;
    mpp::FollowerStatus      finalStatus = mpp::FollowerStatus::Idle;
    double                   maxCross   = 0;  //!< ground truth vs reference [m]
    double                   posErr     = 0;  //!< ground truth, at rest [m]
    double                   headingErr = 0;  //!< ground truth, at rest [rad]
    double                   time       = 0;  //!< [s]
    std::vector<TraceSample> gt;
};

void saveTrace(
    const std::string& name, const mpp::Trajectory& ref, const RunResult& r)
{
    const char* dir = std::getenv("MPP_FOLLOWER_TRACE_DIR");
    if (dir == nullptr) { return; }
    std::ofstream f(std::string(dir) + "/" + name + ".csv");
    f << "kind,t,x,y,phi,v,s,status\n";
    for (const auto& p : ref)
    {
        f << "ref,0," << p.pose.x << "," << p.pose.y << "," << p.pose.phi
          << ",0,0,0\n";
    }
    for (const auto& s : r.gt)
    {
        f << "gt," << s.t << "," << s.pose.x << "," << s.pose.y << ","
          << s.pose.phi << "," << s.v << "," << s.s << "," << s.status << "\n";
    }
}

// Called before every control step, with the time and the last progress:
using on_cycle_t =
    std::function<void(double t, double s, mpp::TrajectoryFollower& f)>;

RunResult runClosedLoop(
    mpp::TrajectoryFollower& f, const mpp::Trajectory& ref,
    const StackConditions& c, const TPose2D& start, double maxTime = 120.0,
    const on_cycle_t& onCycle = {})
{
    constexpr double dtSim   = 0.002;  // [s] plant integration step
    constexpr double tSettle = 2.0;  // [s] simulated after the follower ends

    std::mt19937                           rng(1234);
    std::normal_distribution<double>       gauss(0.0, 1.0);
    std::uniform_real_distribution<double> unif(-1.0, 1.0);

    // Clock origin, so timestamps look like real ones:
    const double t0    = mrpt::Clock::toDouble(mrpt::Clock::now());
    const auto   stamp = [t0](double t)
    { return mrpt::Clock::fromDouble(t0 + t); };

    RunResult r;
    TPose2D   gt   = start;
    TPose2D   odom = start;  // odom frame == map frame at t=0
    double    vAct = 0;
    double    wAct = 0;

    std::deque<TimedPose> gtHist;
    std::deque<TimedPose> odoHist;
    gtHist.push_back({0, gt});
    odoHist.push_back({0, odom});

    // Commands in flight to the platform: (apply time, v, w)
    struct Cmd
    {
        double t;
        double v;
        double w;
    };
    std::deque<Cmd> cmdQueue;
    double          vCmd = 0;
    double          wCmd = 0;

    // Localization: the latest delivered pose and its sensor time.
    bool                                   haveLoc = false;
    TPose2D                                locPose;
    double                                 locSensorT   = 0;
    double                                 locDelivered = 0;
    double                                 nextLocT = 0;  // next sensor time
    std::deque<std::pair<double, TPose2D>> locInFlight;  // (sensor time, pose)

    f.setTrajectory(ref);

    double lastS     = 0;
    double nextCtrlT = 0;
    double endT      = -1;
    double t         = 0;
    for (; t < maxTime; t += dtSim)
    {
        // Localization measurements, delivered after a latency:
        if (t + 1e-9 >= nextLocT)
        {
            TPose2D m = gt;
            m.x += c.loc_noise_xy * gauss(rng);
            m.y += c.loc_noise_xy * gauss(rng);
            m.phi = mrpt::math::wrapToPi(m.phi + c.loc_noise_phi * gauss(rng));
            locInFlight.emplace_back(t, m);
            nextLocT = t + std::max(c.loc_period, dtSim);
        }
        while (!locInFlight.empty() &&
               locInFlight.front().first + c.loc_latency <= t + 1e-9)
        {
            locSensorT   = locInFlight.front().first;
            locPose      = locInFlight.front().second;
            locDelivered = t;
            haveLoc      = true;
            locInFlight.pop_front();
        }

        // Control loop:
        if (endT < 0 && t + 1e-9 >= nextCtrlT && haveLoc)
        {
            mpp::VehicleLocalizationState loc;
            loc.valid    = true;
            loc.pose     = locPose;
            loc.frame_id = "map";
            loc.timestamp =
                stamp(c.loc_true_stamps ? locSensorT : locDelivered);

            const double              tOdo = std::max(0.0, t - c.odo_latency);
            mpp::VehicleOdometryState odo;
            odo.valid                 = true;
            odo.odometry              = poseAt(odoHist, tOdo);
            odo.odometryVelocityLocal = mrpt::math::TTwist2D(vAct, 0, wAct);
            odo.timestamp             = stamp(tOdo);

            if (onCycle) { onCycle(t, lastS, f); }
            const auto out = f.step(loc, odo);
            r.finalStatus  = out.status;
            lastS          = out.arc_length_s;

            double v = 0;
            double w = 0;
            if (out.status == mpp::FollowerStatus::Running &&
                !out.command.points.empty())
            {
                v = out.command.points.front().twist.vx;
                w = out.command.points.front().twist.omega;
            }
            else if (out.status != mpp::FollowerStatus::Running)
            {
                endT = t;  // any terminal status: stop and settle
            }
            cmdQueue.push_back({t + c.cmd_latency, v, w});

            nextCtrlT =
                t + f.params.control_period + c.period_jitter * unif(rng);
        }

        while (!cmdQueue.empty() && cmdQueue.front().t <= t + 1e-9)
        {
            vCmd = cmdQueue.front().v;
            wCmd = cmdQueue.front().w;
            cmdQueue.pop_front();
        }

        // Platform:
        if (c.plant_tau > 0)
        {
            const double a = 1.0 - std::exp(-dtSim / c.plant_tau);
            vAct += (vCmd - vAct) * a;
            wAct += (wCmd - wAct) * a;
        }
        else
        {
            vAct = vCmd;
            wAct = wCmd;
        }
        const TPose2D gtNew = integrate(gt, vAct, wAct, dtSim);

        // Odometry: the true increment, with scale errors.
        const auto inc = mrpt::poses::CPose2D(gtNew) - mrpt::poses::CPose2D(gt);
        const auto incNoisy = mrpt::poses::CPose2D(
            inc.x() * (1.0 + c.odo_scale_error),
            inc.y() * (1.0 + c.odo_scale_error),
            inc.phi() * (1.0 + c.odo_scale_error));
        odom = (mrpt::poses::CPose2D(odom) + incNoisy).asTPose();
        gt   = gtNew;

        gtHist.push_back({t, gt});
        odoHist.push_back({t, odom});
        while (gtHist.size() > 2 && gtHist.front().t < t - 3.0)
        {
            gtHist.pop_front();
        }
        while (odoHist.size() > 2 && odoHist.front().t < t - 3.0)
        {
            odoHist.pop_front();
        }
        if (r.gt.empty() || t - r.gt.back().t >= 0.02)
        {
            r.gt.push_back(
                {t, gt, vAct, lastS, static_cast<int>(r.finalStatus)});
        }

        r.maxCross = std::max(r.maxCross, distToPolyline(ref, {gt.x, gt.y}));

        if (endT >= 0 && t > endT + tSettle)
        {
            r.ended = true;
            break;
        }
    }

    const auto& goal = ref.back().pose;
    r.posErr         = std::hypot(gt.x - goal.x, gt.y - goal.y);
    r.headingErr     = mrpt::math::wrapToPi(gt.phi - goal.phi);
    r.time           = t;
    return r;
}

// Follower parameters as used by the ROS 2 navigation stack for a
// differential-drive robot at 1 m/s.
void setDiffDriveParams(mpp::TrajectoryFollower& f)
{
    auto& p                 = f.params;
    p.max_speed             = 1.0;
    p.max_accel             = 0.8;
    p.max_decel             = 1.0;
    p.max_lateral_accel     = 1.0;
    p.max_omega             = 1.0;
    p.lookahead_max         = 1.5;
    p.lookahead_bend        = mrpt::DEG2RAD(25.0);
    p.min_lookahead_dist    = 0.8;
    p.goal_dist_tol         = 0.15;
    p.goal_ang_tol          = mrpt::DEG2RAD(12.0);
    p.max_cross_track       = 1.0;
    p.off_path_min_duration = 0.5;
    p.arrival_radius        = 0.3;
    p.arrival_ang_tol       = mrpt::DEG2RAD(25.0);
    p.rotate_at_goal        = true;
    p.control_period        = 0.05;
}

// Accuracy demanded from the follower, on the ground truth: deviation from
// the reference path, plus the follower's own goal tolerances.
constexpr double kMaxCrossTrack = 0.15;  // [m]

void expectAccurate(
    const std::string& caseName, const mpp::Trajectory& ref,
    const StackConditions& c, const TPose2D& start)
{
    mpp::TrajectoryFollower f;
    setDiffDriveParams(f);
    // The robot dead time is known (configured) in a real deployment:
    f.params.control_delay = c.odo_latency + c.cmd_latency;
    const auto r           = runClosedLoop(f, ref, c, start);
    saveTrace(caseName + "_" + c.name, ref, r);

    const std::string ctx = caseName + " [" + c.name + "]";
    ASSERT_TRUE(r.ended) << ctx << ": did not finish";
    EXPECT_EQ(r.finalStatus, mpp::FollowerStatus::ReachedGoal)
        << ctx << ": status " << static_cast<int>(r.finalStatus);
    EXPECT_LT(r.maxCross, kMaxCrossTrack) << ctx << ": max deviation";
    EXPECT_LT(r.posErr, f.params.goal_dist_tol)
        << ctx << ": final position error";
    EXPECT_LT(std::abs(r.headingErr), f.params.goal_ang_tol)
        << ctx
        << ": final heading error [deg]: " << mrpt::RAD2DEG(r.headingErr);
}

// ---------------------------------------------------------------------------
// Reference paths
// ---------------------------------------------------------------------------

// Builds a reference path from straight and circular segments, driven
// forward (dir=+1) or in reverse (dir=-1), sampled every `ds` meters.
class PathBuilder
{
   public:
    explicit PathBuilder(const TPose2D& start) : p_(start)
    {
        tr_.emplace_back(p_, 0.0);
    }

    PathBuilder& straight(double len, int dir = 1)
    {
        const int n = std::max(1, static_cast<int>(std::ceil(len / ds_)));
        for (int i = 0; i < n; i++)
        {
            p_ = integrate(p_, dir * len / n, 0, 1.0);
            tr_.emplace_back(p_, 0.0);
        }
        return *this;
    }

    // Turns by `angle` (heading change, + to the left) on a circle of radius R.
    PathBuilder& arc(double R, double angle, int dir = 1)
    {
        const double len = R * std::abs(angle);
        const int    n   = std::max(1, static_cast<int>(std::ceil(len / ds_)));
        for (int i = 0; i < n; i++)
        {
            p_ = integrate(p_, dir * len / n, angle / n, 1.0);
            tr_.emplace_back(p_, 0.0);
        }
        return *this;
    }

    mpp::Trajectory build() const { return tr_; }

   private:
    TPose2D         p_;
    mpp::Trajectory tr_;
    double          ds_ = 0.05;
};

// Diff-drive C-PTGs (forward and reverse) as used for the ROS 2 diffdrive
// robot, with coarser collision grids to keep the test fast.
const char* kJackalPtgs = R"cfg(
[SelfDriving]
min_obstacles_height = 0.0
max_obstacles_height = 2.0
PTG_COUNT = 2

PTG0_Type        = mpp::ptg::DiffDrive_C
PTG0_resolution  = 0.10
PTG0_refDistance = 3.0
PTG0_num_paths   = 61
PTG0_v_max_mps   = 1.0
PTG0_w_max_dps   = 60.0
PTG0_K           = +1.0

PTG1_Type        = mpp::ptg::DiffDrive_C
PTG1_resolution  = 0.10
PTG1_refDistance = 3.0
PTG1_num_paths   = 61
PTG1_v_max_mps   = 1.0
PTG1_w_max_dps   = 60.0
PTG1_K           = -1.0

RobotModel_shape2D_xs = 0.30 0.30 -0.30 -0.30
RobotModel_shape2D_ys = -0.27 0.27 0.27 -0.27
)cfg";

void addWall(
    mrpt::maps::CSimplePointsMap& m, const TPoint2D& a, const TPoint2D& b)
{
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int    n   = std::max(1, static_cast<int>(len / 0.05));
    for (int i = 0; i <= n; i++)
    {
        const double s = static_cast<double>(i) / n;
        m.insertPoint(
            static_cast<float>(a.x + s * (b.x - a.x)),
            static_cast<float>(a.y + s * (b.y - a.y)), 0.0f);
    }
}

// Plans with TPS_Astar and converts the result to a follower reference path,
// the same way navigation nodes do (refined edges sampled every 0.1 s).
mpp::Trajectory planPath(
    const TPose2D& start, const TPose2D& goal,
    const mrpt::maps::CSimplePointsMap::Ptr& obs)
{
    mrpt::config::CConfigFileMemory cfg(kJackalPtgs);
    mpp::PlannerInput               in;
    in.ptgs.initFromConfigFile(cfg, "SelfDriving");
    in.stateStart.pose = start;
    in.stateGoal.state = goal;
    in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

    const auto bb   = obs->boundingBox();
    in.worldBboxMin = {bb.min.x - 1.0, bb.min.y - 1.0, -M_PI};
    in.worldBboxMax = {bb.max.x + 1.0, bb.max.y + 1.0, M_PI};

    mpp::TPS_Astar planner;
    planner.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    planner.params_.grid_resolution_xy              = 0.20;
    planner.params_.grid_resolution_yaw             = mrpt::DEG2RAD(10.0);
    planner.params_.max_ptg_trajectories_to_explore = 20;
    planner.params_.ptg_sample_timestamps           = {1.0, 3.0, 5.0};
    planner.params_.maximumComputationTime          = 30.0;
    planner.params_.use_reeds_shepp_expansion       = true;

    const auto out = planner.plan(in);
    if (!out.success || !out.bestNodeId.has_value()) { return {}; }

    auto [nodes, edges] = out.motionTree.backtrack_path(*out.bestNodeId);
    mpp::refine_trajectory(nodes, edges, in.ptgs);
    const auto samples = mpp::plan_to_trajectory(edges, in.ptgs, 0.1);

    const auto      startPose = mrpt::poses::CPose2D(start);
    mpp::Trajectory ref;
    for (const auto& [t, s] : samples)
    {
        const auto p = startPose + mrpt::poses::CPose2D(s.state.pose);
        ref.emplace_back(p.asTPose(), 0.0);
    }
    return ref;
}

const std::vector<StackConditions>& allStacks()
{
    static const std::vector<StackConditions> s = {
        idealStack(), typicalStack(), overloadedStack()};
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// Hand-built paths
// ---------------------------------------------------------------------------

// Smooth S-curve, forward only, with turning radii well above the minimum.
TEST(FollowerClosedLoop, SCurve)
{
    const auto ref = PathBuilder({0, 0, 0})
                         .straight(2.0)
                         .arc(2.0, M_PI / 2)
                         .arc(2.0, -M_PI / 2)
                         .straight(3.0)
                         .build();
    for (const auto& c : allStacks())
    {
        expectAccurate("s_curve", ref, c, ref.front().pose);
    }
}

// Tight turns at the vehicle minimum turning radius (1 m at 1 m/s, 60 deg/s).
TEST(FollowerClosedLoop, TightTurns)
{
    const auto ref = PathBuilder({0, 0, 0})
                         .straight(1.5)
                         .arc(1.0, M_PI / 2)
                         .straight(1.0)
                         .arc(1.0, -M_PI / 2)
                         .straight(1.5)
                         .build();
    for (const auto& c : allStacks())
    {
        expectAccurate("tight_turns", ref, c, ref.front().pose);
    }
}

// Three-point turn: forward arc, cusp, reverse arc, cusp, forward.
TEST(FollowerClosedLoop, ThreePointTurn)
{
    const auto ref = PathBuilder({0, 0, 0})
                         .straight(1.0)
                         .arc(1.2, M_PI / 2)
                         .arc(1.2, M_PI / 2, -1)
                         .straight(2.0)
                         .build();
    for (const auto& c : allStacks())
    {
        expectAccurate("three_point_turn", ref, c, ref.front().pose);
    }
}

// Short path with a cusp next to the goal, as replanned paths often are:
// back up a little, then drive forward into the goal pose.
TEST(FollowerClosedLoop, ShortManeuverNearGoal)
{
    const auto ref = PathBuilder({0, 0, 0})
                         .arc(1.0, -M_PI / 6, -1)
                         .arc(1.0, M_PI / 3)
                         .straight(0.3)
                         .build();
    for (const auto& c : allStacks())
    {
        expectAccurate("short_maneuver", ref, c, ref.front().pose);
    }
}

// Long approach ending in a reverse maneuver into the goal pose.
TEST(FollowerClosedLoop, ReverseIntoGoal)
{
    const auto ref = PathBuilder({0, 0, 0})
                         .straight(4.0)
                         .arc(1.5, -M_PI / 4)
                         .arc(1.5, M_PI / 2, -1)
                         .straight(1.0, -1)
                         .build();
    for (const auto& c : allStacks())
    {
        expectAccurate("reverse_into_goal", ref, c, ref.front().pose);
    }
}

// ---------------------------------------------------------------------------
// Paths planned by TPS_Astar
// ---------------------------------------------------------------------------

// An L-shaped corridor, 2.4 m wide.
TEST(FollowerClosedLoop, PlannedLCorridor)
{
    auto obs = mrpt::maps::CSimplePointsMap::Create();
    addWall(*obs, {-1, -1.2}, {8.2, -1.2});
    addWall(*obs, {-1, 1.2}, {5.8, 1.2});
    addWall(*obs, {8.2, -1.2}, {8.2, 8});
    addWall(*obs, {5.8, 1.2}, {5.8, 8});
    addWall(*obs, {-1, -1.2}, {-1, 1.2});

    const TPose2D start = {0, 0, 0};
    const TPose2D goal  = {7.0, 6.5, M_PI / 2};
    const auto    ref   = planPath(start, goal, obs);
    ASSERT_GT(ref.size(), 2U) << "planning failed";

    for (const auto& c : allStacks())
    {
        expectAccurate("planned_l_corridor", ref, c, start);
    }
}

// The goal pose faces the opposite way, behind the robot, in a room: the plan
// needs to turn around, usually with cusps.
TEST(FollowerClosedLoop, PlannedTurnAroundInRoom)
{
    auto obs = mrpt::maps::CSimplePointsMap::Create();
    addWall(*obs, {-2, -2}, {6, -2});
    addWall(*obs, {-2, 2}, {6, 2});
    addWall(*obs, {-2, -2}, {-2, 2});
    addWall(*obs, {6, -2}, {6, 2});

    const TPose2D start = {0, 0, 0};
    const TPose2D goal  = {-0.5, 0.8, M_PI};
    const auto    ref   = planPath(start, goal, obs);
    ASSERT_GT(ref.size(), 2U) << "planning failed";

    for (const auto& c : allStacks())
    {
        expectAccurate("planned_turn_around", ref, c, start);
    }
}

// A path update continuing the current path while driving at cruise speed
// (as a replanner sends: the part being driven, then a new tail) must not
// slow the robot down, also if the update drops part of the path already
// driven. Restarting the path instead (setTrajectory) does.
TEST(FollowerClosedLoop, PathUpdateKeepsSpeed)
{
    const auto initial = PathBuilder({0, 0, 0}).straight(14.0).build();
    const auto updated = PathBuilder({0, 0, 0})
                             .straight(7.0)
                             .arc(3.0, M_PI / 2)
                             .straight(3.0)
                             .build();
    // The update without its first 2 m:
    mpp::Trajectory updatedTrimmed;
    for (const auto& p : updated)
    {
        if (p.pose.x >= 2.0 || p.pose.y > 0.1) { updatedTrimmed.push_back(p); }
    }
    ASSERT_LT(updatedTrimmed.size(), updated.size());

    constexpr double kSwapS = 3.5;  // [m]

    auto run =
        [&](const mpp::Trajectory& update, bool keepMotion, double& tSwap)
    {
        mpp::TrajectoryFollower f;
        setDiffDriveParams(f);
        const auto& c          = typicalStack();
        f.params.control_delay = c.odo_latency + c.cmd_latency;
        bool started           = false;
        bool swapped           = false;
        tSwap                  = -1;
        const auto r           = runClosedLoop(
                      f, updated, c, {0, 0, 0}, 120.0,
                      [&](double t, double s, mpp::TrajectoryFollower& fol)
                      {
                if (!started)
                {
                    fol.setTrajectory(initial);
                    started = true;
                }
                if (!swapped && s >= kSwapS)
                {
                    if (keepMotion) { fol.updateTrajectory(update); }
                    else { fol.setTrajectory(update); }
                    swapped = true;
                    tSwap   = t;
                }
            });
        return r;
    };

    // Minimum actual speed in the 2 s after the swap, relative to the speed
    // at the swap:
    auto speedRatioAfterSwap = [](const RunResult& r, double tSwap)
    {
        double vSwap = 0;
        double vMin  = std::numeric_limits<double>::infinity();
        for (const auto& g : r.gt)
        {
            if (g.t <= tSwap) { vSwap = g.v; }
            else if (g.t <= tSwap + 2.0) { vMin = std::min(vMin, g.v); }
        }
        return vSwap > 0 ? vMin / vSwap : 0.0;
    };

    for (const mpp::Trajectory* update :
         std::vector<const mpp::Trajectory*>{&updated, &updatedTrimmed})
    {
        double     tSwap = 0;
        const auto r     = run(*update, true, tSwap);
        saveTrace("path_update", updated, r);
        ASSERT_GT(tSwap, 0);
        ASSERT_TRUE(r.ended);
        EXPECT_EQ(r.finalStatus, mpp::FollowerStatus::ReachedGoal);
        EXPECT_LT(r.maxCross, kMaxCrossTrack);
        EXPECT_GT(speedRatioAfterSwap(r, tSwap), 0.9);
    }

    double     tSwap     = 0;
    const auto restarted = run(updated, false, tSwap);
    EXPECT_LT(speedRatioAfterSwap(restarted, tSwap), 0.5);
}
