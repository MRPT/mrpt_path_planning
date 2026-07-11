/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/robot_shape_sampling.h>
#include <mpp/follow/algos/TrajectoryFollower.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace mpp;

namespace
{
// Exact constant-(v,w) unicycle integration over dt.
mrpt::math::TPose2D integrateUnicycle(
    const mrpt::math::TPose2D& p, double v, double w, double dt)
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
}  // namespace

TrajectoryFollower::TrajectoryFollower()
    : mrpt::system::COutputLogger("TrajectoryFollower")
{
}

// ---------------------------------------------------------------- Parameters
void TrajectoryFollower::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, max_speed);
    MCP_LOAD_OPT(c, max_accel);
    MCP_LOAD_OPT(c, max_decel);
    MCP_LOAD_OPT(c, max_lateral_accel);
    MCP_LOAD_OPT(c, lookahead_min);
    MCP_LOAD_OPT(c, lookahead_max);
    MCP_LOAD_OPT(c, lookahead_time);
    MCP_LOAD_OPT(c, goal_dist_tol);
    MCP_LOAD_OPT_DEG(c, goal_ang_tol);
    MCP_LOAD_OPT(c, max_cross_track);
    MCP_LOAD_OPT(c, control_period);
    MCP_LOAD_OPT(c, horizon);
    MCP_LOAD_OPT(c, sample_period);
    MCP_LOAD_OPT(c, emit_frame);
    MCP_LOAD_OPT(c, safety_margin);
    MCP_LOAD_OPT(c, stop_distance);
    MCP_LOAD_OPT(c, slow_distance);
    MCP_LOAD_OPT(c, safety_horizon);
    MCP_LOAD_OPT(c, reference_lookahead_dist);
    MCP_LOAD_OPT(c, footprint_sample_resolution);
    MCP_LOAD_OPT(c, resume_scale);
    MCP_LOAD_OPT(c, block_timeout);
}

mrpt::containers::yaml TrajectoryFollower::Parameters::as_yaml() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();
    MCP_SAVE(c, max_speed);
    MCP_SAVE(c, max_accel);
    MCP_SAVE(c, max_decel);
    MCP_SAVE(c, max_lateral_accel);
    MCP_SAVE(c, lookahead_min);
    MCP_SAVE(c, lookahead_max);
    MCP_SAVE(c, lookahead_time);
    MCP_SAVE(c, goal_dist_tol);
    MCP_SAVE_DEG(c, goal_ang_tol);
    MCP_SAVE(c, max_cross_track);
    MCP_SAVE(c, control_period);
    MCP_SAVE(c, horizon);
    MCP_SAVE(c, sample_period);
    MCP_SAVE(c, emit_frame);
    MCP_SAVE(c, safety_margin);
    MCP_SAVE(c, stop_distance);
    MCP_SAVE(c, slow_distance);
    MCP_SAVE(c, safety_horizon);
    MCP_SAVE(c, reference_lookahead_dist);
    MCP_SAVE(c, footprint_sample_resolution);
    MCP_SAVE(c, resume_scale);
    MCP_SAVE(c, block_timeout);
    return c;
}

TrajectoryFollower::Parameters TrajectoryFollower::Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    Parameters p;
    p.load_from_yaml(c);
    return p;
}

// ---------------------------------------------------------------- trajectory
void TrajectoryFollower::setTrajectory(const Trajectory& traj)
{
    traj_ = traj;
    cumS_.assign(traj_.size(), 0.0);
    for (std::size_t i = 1; i < traj_.size(); i++)
    {
        const auto& a = traj_[i - 1].pose;
        const auto& b = traj_[i].pose;
        cumS_[i]      = cumS_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
    }
    lastS_        = 0;
    stopped_      = false;
    stoppedSince_ = INVALID_TIMESTAMP;
}

void TrajectoryFollower::reset()
{
    traj_.clear();
    cumS_.clear();
    lastS_        = 0;
    stopped_      = false;
    stoppedSince_ = INVALID_TIMESTAMP;
}

void TrajectoryFollower::setRobotShape(const RobotShape& shape)
{
    shapeSamples_ =
        footprintSamplePoints(shape, params.footprint_sample_resolution);
}

void TrajectoryFollower::setObstacles(const mrpt::maps::CPointsMap& obstacles)
{
    obstacles_.clear();
    obstacles_.insertAnotherMap(&obstacles, mrpt::poses::CPose3D::Identity());
}

void TrajectoryFollower::setObstacles(
    const std::vector<mrpt::math::TPoint2D>& obstacles)
{
    obstacles_.clear();
    for (const auto& p : obstacles) obstacles_.insertPoint(p.x, p.y);
}

// ------------------------------------------------------------------ geometry
TrajectoryFollower::Projection TrajectoryFollower::projectToPath(
    const mrpt::math::TPoint2D& xy, double sHint) const
{
    Projection best;
    double     bestD2 = std::numeric_limits<double>::infinity();

    const double backWindow = 0.5;
    const double fwdWindow  = std::max(3.0, 2.0 * params.lookahead_max);

    // Two passes: a local window around sHint (keeps progress monotone on paths
    // that re-approach themselves), then a full search if nothing matched.
    for (int pass = 0; pass < 2 && !std::isfinite(bestD2); pass++)
    {
        const bool windowed = (pass == 0);
        for (std::size_t i = 0; i + 1 < traj_.size(); i++)
        {
            if (windowed && (cumS_[i + 1] < sHint - backWindow ||
                             cumS_[i] > sHint + fwdWindow))
                continue;

            const auto&  a    = traj_[i].pose;
            const auto&  b    = traj_[i + 1].pose;
            const double sx   = b.x - a.x;
            const double sy   = b.y - a.y;
            const double len2 = sx * sx + sy * sy;
            if (len2 < 1e-9) continue;

            double t        = ((xy.x - a.x) * sx + (xy.y - a.y) * sy) / len2;
            t               = std::clamp(t, 0.0, 1.0);
            const double px = a.x + t * sx;
            const double py = a.y + t * sy;
            const double d2 =
                (xy.x - px) * (xy.x - px) + (xy.y - py) * (xy.y - py);
            if (d2 < bestD2)
            {
                bestD2             = d2;
                best.s             = cumS_[i] + t * std::sqrt(len2);
                best.path_heading  = std::atan2(sy, sx);
                const double cross = sx * (xy.y - a.y) - sy * (xy.x - a.x);
                best.cross_track   = (cross >= 0 ? 1.0 : -1.0) * std::sqrt(d2);
            }
        }
    }
    if (!std::isfinite(bestD2)) best.s = sHint;
    return best;
}

mrpt::math::TPoint2D TrajectoryFollower::pointAtArc(double s) const
{
    if (traj_.empty()) return {0, 0};
    s = std::clamp(s, 0.0, totalLength());
    // find segment [i, i+1] containing s
    std::size_t i = 0;
    while (i + 2 < traj_.size() && cumS_[i + 1] < s) i++;
    const auto&  a      = traj_[i].pose;
    const auto&  b      = traj_[i + 1].pose;
    const double segLen = cumS_[i + 1] - cumS_[i];
    const double t      = segLen > 1e-9 ? (s - cumS_[i]) / segLen : 0.0;
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

mrpt::math::TPose2D TrajectoryFollower::poseAtArc(double s) const
{
    if (traj_.empty()) return {0, 0, 0};
    s             = std::clamp(s, 0.0, totalLength());
    std::size_t i = 0;
    while (i + 2 < traj_.size() && cumS_[i + 1] < s) i++;
    const auto&  a       = traj_[i].pose;
    const auto&  b       = traj_[i + 1].pose;
    const double segLen  = cumS_[i + 1] - cumS_[i];
    const double t       = segLen > 1e-9 ? (s - cumS_[i]) / segLen : 0.0;
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), heading};
}

double TrajectoryFollower::speedCapAt(double s) const
{
    if (traj_.empty()) return params.max_speed;
    s             = std::clamp(s, 0.0, totalLength());
    std::size_t i = 0;
    while (i + 2 < traj_.size() && cumS_[i + 1] < s) i++;
    const double segLen = cumS_[i + 1] - cumS_[i];
    const double t      = segLen > 1e-9 ? (s - cumS_[i]) / segLen : 0.0;
    auto         eff = [&](double v) { return v <= 0 ? params.max_speed : v; };
    const double v0  = eff(traj_[i].target_speed);
    const double v1  = eff(traj_[i + 1].target_speed);
    const double cap = v0 + t * (v1 - v0);
    return std::min(cap, params.max_speed);
}

// ------------------------------------------------------------------- pursuit
TrajectoryFollower::Command TrajectoryFollower::pursuit(
    const mrpt::math::TPose2D& fromPose, double currentV, double sHint,
    double dt, double speedScale) const
{
    Command          out;
    const Projection proj = projectToPath({fromPose.x, fromPose.y}, sHint);

    const double L = std::clamp(
        params.lookahead_time * currentV, params.lookahead_min,
        params.lookahead_max);
    out.lookahead = pointAtArc(proj.s + L);

    // Lookahead in robot frame.
    const double dx = out.lookahead.x - fromPose.x;
    const double dy = out.lookahead.y - fromPose.y;
    const double c  = std::cos(fromPose.phi);
    const double sn = std::sin(fromPose.phi);
    const double xr = c * dx + sn * dy;
    const double yr = -sn * dx + c * dy;
    const double Ld = std::hypot(xr, yr);

    const double curv = Ld > 1e-3 ? 2.0 * yr / (Ld * Ld) : 0.0;

    // Speed caps: profile, curvature (lateral accel), and decel-to-goal.
    const double remaining = std::max(0.0, totalLength() - proj.s);
    double       cap       = speedCapAt(proj.s);
    if (std::abs(curv) > 1e-3)
        cap =
            std::min(cap, std::sqrt(params.max_lateral_accel / std::abs(curv)));
    cap = std::min(cap, std::sqrt(2.0 * params.max_decel * remaining));

    // Predictive-safety speed scale (caps the target before rate-limiting so
    // decel stays bounded by max_decel).
    cap *= std::clamp(speedScale, 0.0, 1.0);

    // Rate-limit the speed toward the cap.
    double v = std::clamp(
        cap, currentV - params.max_decel * dt,
        currentV + params.max_accel * dt);
    v = std::max(0.0, v);

    out.v     = v;
    out.omega = v * curv;
    return out;
}

// -------------------------------------------------------------------- safety
double TrajectoryFollower::footprintClearance(
    const mrpt::math::TPose2D& p) const
{
    if (obstacles_.empty()) return std::numeric_limits<double>::infinity();

    const double c = std::cos(p.phi);
    const double s = std::sin(p.phi);

    // With no footprint set, fall back to the reference point only.
    auto clearanceAt = [&](double lx, double ly) -> double
    {
        const double wx = p.x + lx * c - ly * s;
        const double wy = p.y + lx * s + ly * c;
        return std::sqrt(static_cast<double>(
            obstacles_.kdTreeClosestPoint2DsqrError(wx, wy)));
    };

    if (shapeSamples_.empty()) return clearanceAt(0.0, 0.0);

    double best = std::numeric_limits<double>::infinity();
    for (const auto& v : shapeSamples_)
        best = std::min(best, clearanceAt(v.x, v.y));
    return best;
}

double TrajectoryFollower::forecastContactDistance(
    const mrpt::math::TPose2D& startPose, double startV, double startS) const
{
    double              L = 0;
    mrpt::math::TPose2D p = startPose;
    double              v = std::max(0.0, startV);
    double              s = startS;

    const int nSteps = std::max(
        1, static_cast<int>(
               std::ceil(params.safety_horizon / params.sample_period)));

    for (int k = 0; k <= nSteps; k++)
    {
        if (footprintClearance(p) <= params.safety_margin) return L;

        const Command cmd = pursuit(p, v, s, params.sample_period, 1.0);
        const mrpt::math::TPose2D pNext =
            integrateUnicycle(p, cmd.v, cmd.omega, params.sample_period);

        L += std::hypot(pNext.x - p.x, pNext.y - p.y);
        p = pNext;
        v = cmd.v;
        s = projectToPath({p.x, p.y}, s).s;

        if (totalLength() - s <= params.goal_dist_tol) break;
    }
    return std::numeric_limits<double>::infinity();
}

double TrajectoryFollower::referenceContactDistance(double startS) const
{
    const double step = std::max(0.02, params.footprint_sample_resolution);
    for (double ds = 0; ds <= params.reference_lookahead_dist; ds += step)
    {
        const double s = startS + ds;
        if (s > totalLength()) break;
        if (footprintClearance(poseAtArc(s)) <= params.safety_margin) return ds;
    }
    return std::numeric_limits<double>::infinity();
}

double TrajectoryFollower::contactDistanceToScale(double d) const
{
    if (!std::isfinite(d)) return 1.0;
    const double span =
        std::max(1e-3, params.slow_distance - params.stop_distance);
    return std::clamp((d - params.stop_distance) / span, 0.0, 1.0);
}

// ---------------------------------------------------------------------- step
TrajectoryFollower::Output TrajectoryFollower::step(
    const VehicleLocalizationState& loc, const VehicleOdometryState& odo)
{
    Output out;
    if (!hasTrajectory())
    {
        out.status = FollowerStatus::Idle;
        return out;
    }

    const Projection proj = projectToPath({loc.pose.x, loc.pose.y}, lastS_);
    lastS_                = proj.s;

    // Heading at the path end (last segment direction).
    const auto&  pEndA      = traj_[traj_.size() - 2].pose;
    const auto&  pEndB      = traj_.back().pose;
    const double endHeading = std::atan2(pEndB.y - pEndA.y, pEndB.x - pEndA.x);

    out.arc_length_s    = proj.s;
    out.cross_track_err = proj.cross_track;
    out.heading_err     = mrpt::math::wrapToPi(loc.pose.phi - endHeading);

    const double remaining = totalLength() - proj.s;

    // Goal reached?
    if (remaining <= params.goal_dist_tol &&
        std::abs(out.heading_err) <= params.goal_ang_tol)
    {
        out.status = FollowerStatus::ReachedGoal;
        return out;  // empty command => node stops
    }

    out.status = std::abs(proj.cross_track) > params.max_cross_track
                     ? FollowerStatus::OffPathExceeded
                     : FollowerStatus::Running;

    double predV = odo.valid ? odo.odometryVelocityLocal.vx : 0.0;
    predV        = std::max(0.0, predV);

    // Predictive safety: sweep the footprint over the command forecast and the
    // reference path ahead, and scale the commanded speed toward a stop before
    // contact. A hysteresis latch avoids chattering; a sustained stop reports
    // Blocked. Inert when no obstacles/footprint are set.
    double scale = 1.0;
    if (!obstacles_.empty())
    {
        const double dFwd = forecastContactDistance(loc.pose, predV, proj.s);
        const double dRef = referenceContactDistance(proj.s);
        scale             = std::min(
                        contactDistanceToScale(dFwd), contactDistanceToScale(dRef));
    }

    const auto nowStamp =
        loc.timestamp != INVALID_TIMESTAMP ? loc.timestamp : mrpt::Clock::now();
    if (stopped_)
    {
        if (scale >= params.resume_scale)
        {
            stopped_      = false;
            stoppedSince_ = INVALID_TIMESTAMP;
        }
        else
            scale = 0.0;
    }
    if (!stopped_ && scale <= 1e-3)
    {
        stopped_      = true;
        stoppedSince_ = nowStamp;
    }
    if (stopped_ && out.status != FollowerStatus::OffPathExceeded &&
        stoppedSince_ != INVALID_TIMESTAMP &&
        mrpt::system::timeDifference(stoppedSince_, nowStamp) >=
            params.block_timeout)
        out.status = FollowerStatus::Blocked;

    out.safety_scale = scale;

    // map->odom correction so the emitted chunk is smooth despite
    // relocalization.
    const mrpt::poses::CPose2D map2odom =
        mrpt::poses::CPose2D(odo.odometry) +
        (mrpt::poses::CPose2D() - mrpt::poses::CPose2D(loc.pose));

    mrpt::math::TPose2D predPose = loc.pose;
    double              predS    = proj.s;

    out.command.frame_id = params.emit_frame;
    out.command.stamp    = loc.timestamp;

    const int nSamples = std::max(
        1, static_cast<int>(std::ceil(params.horizon / params.sample_period)));

    for (int k = 0; k <= nSamples; k++)
    {
        const Command cmd =
            pursuit(predPose, predV, predS, params.sample_period, scale);

        TrajSample smp;
        smp.t = k * params.sample_period;
        const mrpt::poses::CPose2D poseOdom =
            map2odom + mrpt::poses::CPose2D(predPose);
        smp.pose        = poseOdom.asTPose();
        smp.twist       = {cmd.v, 0.0, cmd.omega};
        smp.speed_scale = scale;
        out.command.points.push_back(smp);

        if (k == 0)
        {
            out.target_speed    = cmd.v;
            out.lookahead_point = cmd.lookahead;
        }

        // Advance the forecast.
        predPose =
            integrateUnicycle(predPose, cmd.v, cmd.omega, params.sample_period);
        predV = cmd.v;
        predS = projectToPath({predPose.x, predPose.y}, predS).s;

        if (totalLength() - predS <= params.goal_dist_tol) break;
    }

    return out;
}
