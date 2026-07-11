/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/follow/algos/TrajectoryFollower.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>

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
    lastS_ = 0;
}

void TrajectoryFollower::reset()
{
    traj_.clear();
    cumS_.clear();
    lastS_ = 0;
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
    double dt) const
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

    // Rate-limit the speed toward the cap.
    double v = std::clamp(
        cap, currentV - params.max_decel * dt,
        currentV + params.max_accel * dt);
    v = std::max(0.0, v);

    out.v     = v;
    out.omega = v * curv;
    return out;
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

    // map->odom correction so the emitted chunk is smooth despite
    // relocalization.
    const mrpt::poses::CPose2D map2odom =
        mrpt::poses::CPose2D(odo.odometry) +
        (mrpt::poses::CPose2D() - mrpt::poses::CPose2D(loc.pose));

    double predV = odo.valid ? odo.odometryVelocityLocal.vx : 0.0;
    predV        = std::max(0.0, predV);
    mrpt::math::TPose2D predPose = loc.pose;
    double              predS    = proj.s;

    out.command.frame_id = params.emit_frame;
    out.command.stamp    = loc.timestamp;

    const int nSamples = std::max(
        1, static_cast<int>(std::ceil(params.horizon / params.sample_period)));

    for (int k = 0; k <= nSamples; k++)
    {
        const Command cmd =
            pursuit(predPose, predV, predS, params.sample_period);

        TrajSample smp;
        smp.t = k * params.sample_period;
        const mrpt::poses::CPose2D poseOdom =
            map2odom + mrpt::poses::CPose2D(predPose);
        smp.pose        = poseOdom.asTPose();
        smp.twist       = {cmd.v, 0.0, cmd.omega};
        smp.speed_scale = 1.0;  // no safety scaling in the pursuit core yet
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
