/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/TrajectoryFollower.h>
#include <mpp/data/robot_shape_sampling.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
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
    MCP_LOAD_OPT(c, min_turn_radius);
    MCP_LOAD_OPT(c, max_omega_rate);
    MCP_LOAD_OPT(c, max_curvature_rate);
    MCP_LOAD_OPT(c, lookahead_max);
    MCP_LOAD_OPT_DEG(c, lookahead_bend);
    MCP_LOAD_OPT(c, min_lookahead_arc);
    MCP_LOAD_OPT(c, min_lookahead_dist);
    MCP_LOAD_OPT(c, goal_dist_tol);
    MCP_LOAD_OPT_DEG(c, goal_ang_tol);
    MCP_LOAD_OPT(c, max_cross_track);
    MCP_LOAD_OPT(c, off_path_min_duration);
    MCP_LOAD_OPT(c, anchor_time_constant);
    MCP_LOAD_OPT(c, anchor_max_lin_rate);
    MCP_LOAD_OPT(c, anchor_max_ang_rate);
    MCP_LOAD_OPT(c, anchor_max_lin_divergence);
    MCP_LOAD_OPT(c, anchor_max_ang_divergence);
    MCP_LOAD_OPT(c, arrival_radius);
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
    MCP_SAVE(c, min_turn_radius);
    MCP_SAVE(c, max_omega_rate);
    MCP_SAVE(c, max_curvature_rate);
    MCP_SAVE(c, lookahead_max);
    MCP_SAVE_DEG(c, lookahead_bend);
    MCP_SAVE(c, min_lookahead_arc);
    MCP_SAVE(c, min_lookahead_dist);
    MCP_SAVE(c, goal_dist_tol);
    MCP_SAVE_DEG(c, goal_ang_tol);
    MCP_SAVE(c, max_cross_track);
    MCP_SAVE(c, off_path_min_duration);
    MCP_SAVE(c, anchor_time_constant);
    MCP_SAVE(c, anchor_max_lin_rate);
    MCP_SAVE(c, anchor_max_ang_rate);
    MCP_SAVE(c, anchor_max_lin_divergence);
    MCP_SAVE(c, anchor_max_ang_divergence);
    MCP_SAVE(c, arrival_radius);
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
    lastS_              = 0;
    currentInterval_    = 0;
    lastCommandedSpeed_ = 0;
    lastCommandedOmega_ = 0;
    lastCommandedCurv_  = 0;
    stopped_            = false;
    stoppedSince_       = INVALID_TIMESTAMP;
    offPathSince_       = INVALID_TIMESTAMP;
    arrived_            = false;
    minDistToGoal_      = std::numeric_limits<double>::infinity();
    ctrlPoseInit_       = false;

    // Cusp arc-lengths: points where the path travel direction reverses (a
    // differential-drive planner backs up then drives forward, etc.). The
    // pursuit must not look *across* a cusp, or the lookahead jumps to the
    // doubling-back branch and commands an erratic, over-rotating curvature.
    cuspS_.clear();
    cuspIdx_.clear();
    for (std::size_t i = 1; i + 1 < traj_.size(); i++)
    {
        const auto&  a  = traj_[i - 1].pose;
        const auto&  b  = traj_[i].pose;
        const auto&  c2 = traj_[i + 1].pose;
        const double ux = b.x - a.x, uy = b.y - a.y;
        const double wx = c2.x - b.x, wy = c2.y - b.y;
        if (ux * wx + uy * wy < 0.0)
        {
            cuspS_.push_back(cumS_[i]);
            cuspIdx_.push_back(i);
        }
    }

    // Driving gear per cusp-bounded interval, decided independently for each
    // interval's own end boundary (no dependency between intervals): reverse
    // when the vehicle's recorded heading approaching that boundary is
    // opposed to the boundary's own approach direction (e.g. a
    // differential-drive planner backs the robot in), forward when it
    // matches (an open-space approach the robot can drive nose-first).
    // Measured over a <=0.7 m window so a jagged micro-tail does not decide
    // it; the window is anchored at the boundary and reaches backward as far
    // as needed (even past an earlier cusp) rather than being clipped to the
    // interval's own extent, so a short interval still gets a robust
    // decision instead of being dominated by its own noise.
    //
    // The heading compared against is the *previous* knot's, not the
    // boundary knot's own: at an intermediate cusp, comparing against the
    // cusp knot's own heading would, for a planner that reports each
    // waypoint's heading as its tangent to the *next* leg (a valid
    // convention some callers use), spuriously look like a reversal there
    // even for an ordinary same-gear sharp corner (its heading is that of
    // the *incoming* leg, not the outgoing one the corner is actually
    // turning into). The previous knot is unambiguously still inside this
    // interval under either heading convention.
    gearPerInterval_.assign(cuspS_.size() + 1, 1.0);
    if (traj_.size() >= 2)
    {
        for (std::size_t k = 0; k < gearPerInterval_.size(); k++)
        {
            const bool        isLast = k == cuspS_.size();
            const double      sEnd   = isLast ? totalLength() : cuspS_[k];
            const std::size_t endHIdx =
                isLast ? traj_.size() - 1 : cuspIdx_[k] - 1;
            const double endH  = traj_[endHIdx].pose.phi;
            const double w     = std::min(sEnd, 0.7);
            const auto   pB    = pointAtArc(sEnd);
            const auto   pA    = pointAtArc(sEnd - w);
            const double chord = std::hypot(pB.x - pA.x, pB.y - pA.y);
            const double tang =
                chord > 1e-3 ? std::atan2(pB.y - pA.y, pB.x - pA.x) : endH;
            gearPerInterval_[k] =
                std::cos(mrpt::math::wrapToPi(endH - tang)) >= 0.0 ? 1.0 : -1.0;
        }
    }
}

double TrajectoryFollower::advanceGear(double s)
{
    // Monotonic and latched: once progress reaches a cusp's own arc-length,
    // permanently commit to the next interval and never step back, even if a
    // later cycle's projection lands slightly earlier again. This matters
    // beyond the instant of crossing: on a path that loops back close to
    // itself (the very shape a cusp maneuver produces), a robot that has
    // drifted off-track can have its nearest-point projection clamp to the
    // cusp vertex from *both* neighboring segments over a whole range of
    // nearby positions (projectToPath's per-segment `t` clamps to that
    // shared endpoint) -- and the vehicle naturally slows toward ~0 right at
    // a cusp by design (see the decel-to-cusp cap in pursuit()), so it can
    // sit there, straddling the arc-length by less than a millimeter, for
    // many cycles. A stateless "which interval is s in" lookup would flip
    // back and forth across that millimeter every cycle (each flip reversing
    // the commanded direction) and never actually cross; latching makes the
    // crossing a one-way, one-time decision instead.
    while (currentInterval_ < cuspS_.size() &&
           s >= cuspS_[currentInterval_] - 1e-6)
    {
        currentInterval_++;
    }
    return gearPerInterval_.empty() ? 1.0 : gearPerInterval_[currentInterval_];
}

void TrajectoryFollower::reset()
{
    traj_.clear();
    cumS_.clear();
    cuspS_.clear();
    cuspIdx_.clear();
    gearPerInterval_.clear();
    currentInterval_    = 0;
    lastS_              = 0;
    lastCommandedSpeed_ = 0;
    lastCommandedOmega_ = 0;
    lastCommandedCurv_  = 0;
    stopped_            = false;
    stoppedSince_       = INVALID_TIMESTAMP;
    offPathSince_       = INVALID_TIMESTAMP;
    arrived_            = false;
    minDistToGoal_      = std::numeric_limits<double>::infinity();
    ctrlPoseInit_       = false;
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

double TrajectoryFollower::adaptiveLookaheadS(double sStart, double capS) const
{
    // Arc-length floor so the lookahead never collapses onto the robot; with
    // an arbitrarily short lookahead the pure-pursuit curvature diverges. The
    // min_turn_radius clamp and this floor together bound the commanded turn.
    const double kFloor = params.min_lookahead_arc > 0
                              ? params.min_lookahead_arc
                              : 0.3;  // [m]
    const double total  = totalLength();
    const double sFloor = sStart + kFloor;
    if (capS <= sFloor) return std::min(std::max(capS, sStart), total);

    // Segment tangent (heading of segment [k, k+1]).
    auto tangentOf = [&](std::size_t k)
    {
        const auto& a = traj_[k].pose;
        const auto& b = traj_[k + 1].pose;
        return std::atan2(b.y - a.y, b.x - a.x);
    };

    // March knot by knot from the segment containing sStart, accumulating the
    // absolute turned angle. Stop where the path has bent by lookahead_bend
    // (past the floor), else at the cap.
    std::size_t i = 0;
    while (i + 2 < traj_.size() && cumS_[i + 1] < sStart) i++;

    double accum    = 0.0;
    double prevTang = tangentOf(i);
    for (std::size_t j = i + 1; j + 1 < traj_.size(); j++)
    {
        const double sj = cumS_[j];
        if (sj >= capS) break;
        const double tang = tangentOf(j);
        accum += std::abs(mrpt::math::wrapToPi(tang - prevTang));
        prevTang = tang;
        // Bend reached: place the lookahead here, but never below the floor --
        // clamp it up to sFloor rather than skipping past the bend (skipping
        // would let the lookahead leap beyond a corner the robot is right on
        // top of, which cuts the corner instead of tracing it).
        if (accum >= params.lookahead_bend)
        {
            return std::clamp(sj, sFloor, capS);
        }
    }
    return std::min(capS, total);
}

// ------------------------------------------------------------------- pursuit
TrajectoryFollower::Command TrajectoryFollower::pursuit(
    const mrpt::math::TPose2D& fromPose, double currentV, double currentOmega,
    double currentCurv, double sHint, double dt, double gear,
    double speedScale) const
{
    Command          out;
    const Projection proj = projectToPath({fromPose.x, fromPose.y}, sHint);

    // Do not look past the next cusp: on a path that doubles back, a lookahead
    // reaching into the reversed branch yields a huge spurious curvature (the
    // robot tries to spin toward a point it should reach by reversing gear, not
    // by turning). Clamp the lookahead arc-length to the next direction change.
    // Bounded via currentInterval_ (the same latch `gear` was already read
    // from), not by re-scanning cuspS_ against `proj.s`: projectToPath's
    // windowed search can legitimately let `proj.s` drift back below a cusp
    // the vehicle has already committed past (its own nearest-point search
    // window, not a progress guarantee), which would otherwise re-clamp the
    // lookahead onto a cusp already behind the vehicle and pin it there.
    const double nextCuspS = currentInterval_ < cuspS_.size()
                                 ? cuspS_[currentInterval_]
                                 : std::numeric_limits<double>::infinity();
    // Curvature-adaptive lookahead within the [proj.s, cap] arc-length window,
    // where the cap is the shorter of the max travel and the next cusp.
    const double capS = std::min(proj.s + params.lookahead_max, nextCuspS);
    double       lookaheadS = adaptiveLookaheadS(proj.s, capS);
    out.lookahead           = pointAtArc(lookaheadS);

    // Lookahead in robot frame.
    const double c            = std::cos(fromPose.phi);
    const double sn           = std::sin(fromPose.phi);
    auto         toRobotFrame = [&](const mrpt::math::TPoint2D& p)
    {
        const double dx = p.x - fromPose.x;
        const double dy = p.y - fromPose.y;
        return mrpt::math::TPoint2D(c * dx + sn * dy, -sn * dx + c * dy);
    };
    mrpt::math::TPoint2D lr = toRobotFrame(out.lookahead);
    double               Ld = lr.norm();

    // Euclidean lookahead floor (enforced everywhere along the path, not
    // only at the goal). The pure-pursuit correction gain scales as ~2/Ld^2,
    // so a collapsed Euclidean lookahead turns centimeter-level localization
    // noise into full-authority steering swings (a collapsed Ld also swings
    // the *bearing* to the target far more per unit of Cartesian
    // perturbation); and if Ld ends up inside the vehicle's own (clamped)
    // minimum turning circle the point becomes geometrically unreachable by
    // turning at all. Two stages:
    //  1. advance the lookahead further along the path (past the bend cap,
    //     still bounded by capS = max travel / next cusp);
    //  2. if pinned at capS and still too close (closing on the goal or a
    //     cusp, or a path curling back on itself), extrapolate past the pin
    //     along a smoothed trailing-chord direction (robust to a few noisy
    //     terminal knots, unlike the raw last-segment secant).
    if (params.min_lookahead_dist > 0.0)
    {
        const double minLd =
            std::max(params.min_lookahead_dist, params.min_turn_radius);
        while (Ld < minLd && lookaheadS + 1e-6 < capS)
        {
            lookaheadS    = std::min(capS, lookaheadS + 0.1);
            out.lookahead = pointAtArc(lookaheadS);
            lr            = toRobotFrame(out.lookahead);
            Ld            = lr.norm();
        }
        if (Ld < minLd)
        {
            // Pinned at capS (goal, cusp, or max travel) and still inside
            // the floor circle: extrapolate along the trailing chord.
            const double     sPin       = std::min(capS, totalLength());
            constexpr double kEndWindow = 0.5;  // [m]
            const double     w          = std::min(sPin, kEndWindow);
            const mrpt::math::TPoint2D pEndA = pointAtArc(sPin - w);
            double                     ex    = out.lookahead.x - pEndA.x;
            double                     ey    = out.lookahead.y - pEndA.y;
            const double               eLen  = std::hypot(ex, ey);
            if (eLen > 1e-6)
            {
                ex /= eLen;
                ey /= eLen;
            }
            else
            {
                // Degenerate trailing chord: fall back to the path tangent
                // just before the pin (the reference's own final heading may
                // be opposite the travel direction on a reverse approach).
                const double pinHeading =
                    poseAtArc(std::max(0.0, sPin - 1e-3)).phi;
                ex = std::cos(pinHeading);
                ey = std::sin(pinHeading);
            }
            // Solve for the travel `t` along the ray (pin + t*dir) that puts
            // it exactly `minLd` from the vehicle (a ray-circle intersection),
            // rather than just adding `minLd - Ld` along the direction -- that
            // naive offset only grows Ld 1:1 when the direction points
            // straight away from the vehicle, which is not generally true
            // here. `e` = vehicle->pin; solve |e + t*dir|^2 = minLd^2 (dir is
            // unit length) and take the positive root (there is exactly one,
            // since |e| = Ld < minLd means the pin is already inside the
            // target circle).
            const double ex0  = out.lookahead.x - fromPose.x;
            const double ey0  = out.lookahead.y - fromPose.y;
            const double b    = ex0 * ex + ey0 * ey;  // e . dir
            const double disc = b * b + (minLd * minLd - Ld * Ld);
            const double t    = -b + std::sqrt(std::max(0.0, disc));
            out.lookahead.x += t * ex;
            out.lookahead.y += t * ey;
            lr = toRobotFrame(out.lookahead);
            Ld = lr.norm();
        }
    }
    const double yr = lr.y;

    // Pure-pursuit curvature; the same circle is driven forward or in reverse,
    // the sign of the commanded speed (via `gear`) sets the travel direction.
    double curv = Ld > 1e-3 ? 2.0 * yr / (Ld * Ld) : 0.0;
    // Desired (unclamped) curvature, kept for the lateral-accel speed cap
    // below: when the turn the pursuit geometry actually wants is tighter
    // than the vehicle can make, clamping curv down (next) still leaves the
    // vehicle needing to travel a wide arc around the lookahead point: the
    // speed must be capped for *that* wide, fast sweep, not for the gentler
    // clamped curvature it ends up commanding, or it cruises through a sharp
    // turn too fast and overshoots the reference further than necessary.
    const double curvDesired = curv;

    // Clamp to the vehicle's own steering-limited minimum turn radius (if
    // set): unlike the lateral-accel cap below, which only trades off speed
    // for curvature and so still lets an arbitrarily tight (just slow) turn
    // through, this bounds the curvature itself so the follower never
    // commands something the vehicle physically cannot track.
    if (params.min_turn_radius > 0.0)
    {
        const double maxCurv = 1.0 / params.min_turn_radius;
        curv                 = std::clamp(curv, -maxCurv, maxCurv);
    }

    // Rate-limit the commanded curvature itself (finite steering slew): the
    // curvature-domain counterpart of the omega rate limit below, preferred
    // for steered platforms since it keeps (v, omega) on one consistent
    // circle while the speed profile changes v, and does not slew the
    // legitimate instantaneous omega sign flip at a cusp (constant
    // curvature, v reversing).
    if (params.max_curvature_rate > 0.0)
    {
        const double maxDCurv = params.max_curvature_rate * dt;
        curv                  = std::clamp(
                             curv, currentCurv - maxDCurv, currentCurv + maxDCurv);
    }
    out.curv = curv;

    // Speed caps (magnitude): profile, curvature (lateral accel),
    // decel-to-goal/decel-to-next-cusp. Both use the straight-line distance to
    // the target point (the arc-length projection saturates before the robot
    // physically arrives and would park it short); a terminal stop latch in
    // step() prevents any runaway once the robot has settled near the goal. A
    // cusp is a sub-goal in the same sense: the path reverses travel direction
    // there, so the vehicle must have slowed to near-zero by then too, or it
    // cruises up to the cusp at speed and then has to take the sharp
    // just-past-cusp turn while still fast.
    const mrpt::math::TPoint2D goalPt = pointAtArc(totalLength());
    const double               distToGoal =
        std::hypot(goalPt.x - fromPose.x, goalPt.y - fromPose.y);
    double distToStop = distToGoal;
    if (std::isfinite(nextCuspS))
    {
        const mrpt::math::TPoint2D cuspPt = pointAtArc(nextCuspS);
        distToStop                        = std::min(
                                   distToStop,
                                   std::hypot(cuspPt.x - fromPose.x, cuspPt.y - fromPose.y));
    }
    double cap = speedCapAt(proj.s);
    if (std::abs(curvDesired) > 1e-3)
        cap = std::min(
            cap, std::sqrt(params.max_lateral_accel / std::abs(curvDesired)));
    cap = std::min(cap, std::sqrt(2.0 * params.max_decel * distToStop));

    // Predictive-safety speed scale (caps the target before rate-limiting so
    // decel stays bounded by max_decel).
    cap *= std::clamp(speedScale, 0.0, 1.0);

    // Rate-limit the *signed* speed toward the gear-directed target. `sCur` is
    // the current speed measured in the gear's forward sense; if it is negative
    // the robot is still moving the other way (a cusp), so brake to zero before
    // driving off in the new gear rather than step-changing direction.
    const double sCur    = gear * currentV;
    const double sTarget = cap;  // desired magnitude in the gear frame
    double       sNew;
    if (sCur < 0.0)
        sNew = std::min(0.0, sCur + params.max_decel * dt);
    else if (sTarget >= sCur)
        sNew = std::min(sTarget, sCur + params.max_accel * dt);
    else
        sNew = std::max(sTarget, sCur - params.max_decel * dt);

    out.v     = gear * sNew;
    out.omega = out.v * curv;
    const double omegaDesired = out.omega;

    // Rate-limit the commanded angular velocity itself: pure pursuit is a
    // memoryless geometric controller, so a lookahead-point jump (path noise,
    // a lookahead pinning to the goal, a cusp) can otherwise step omega by
    // more than a real steering actuator can track in one control period.
    // Chasing that lag with further full-authority corrections is what
    // produces oscillation/"dancing" rather than convergence; this keeps the
    // commanded omega within a rate the actuator can actually follow.
    if (params.max_omega_rate > 0.0)
    {
        const double maxDOmega = params.max_omega_rate * dt;
        out.omega              = std::clamp(
                         out.omega, currentOmega - maxDOmega, currentOmega + maxDOmega);
    }

    // Debug trace (off unless the caller raises the COutputLogger verbosity
    // to LVL_DEBUG, e.g. via the ROS node's `follower_debug_trace` param):
    // one line per pursuit() call, i.e. per predicted sample within a
    // control cycle's horizon rollout (~horizon/sample_period calls), not
    // just the first (k==0, actually-commanded) one -- useful to see how the
    // forecast curvature/speed evolve, not only what gets published.
    // `rosT` is the wall-clock UNIX-epoch time of this call (pursuit() has
    // no localization timestamp of its own to reuse, unlike [step]'s
    // `nowStamp`); still directly comparable to a recorded bag's topic
    // timestamps when use_sim_time is false.
    MRPT_LOG_DEBUG_STREAM(
        "[pursuit] rosT=" << std::fixed << std::setprecision(6)
                          << mrpt::Clock::nowDouble() << " s=" << proj.s
                          << " gear=" << gear << " Ld=" << Ld << " yr=" << yr
                          << " curvDesired=" << curvDesired << " curvClamped="
                          << curv << " cap=" << cap << " sCur=" << sCur
                          << " sTarget=" << sTarget << " sNew=" << sNew
                          << " v=" << out.v << " omegaDesired=" << omegaDesired
                          << " omegaOut=" << out.omega
                          << " distToStop=" << distToStop);

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
    const mrpt::math::TPose2D& startPose, double startV, double startOmega,
    double startCurv, double startS, double gear) const
{
    double              L  = 0;
    mrpt::math::TPose2D p  = startPose;
    double              v  = startV;  // signed (reverse forecasts back up)
    double              om = startOmega;
    double              cv = startCurv;
    double              s  = startS;

    const int nSteps = std::max(
        1, static_cast<int>(
               std::ceil(params.safety_horizon / params.sample_period)));

    for (int k = 0; k <= nSteps; k++)
    {
        if (footprintClearance(p) <= params.safety_margin) return L;

        const Command cmd =
            pursuit(p, v, om, cv, s, params.sample_period, gear, 1.0);
        const mrpt::math::TPose2D pNext =
            integrateUnicycle(p, cmd.v, cmd.omega, params.sample_period);

        L += std::hypot(pNext.x - p.x, pNext.y - p.y);
        p  = pNext;
        v  = cmd.v;
        om = cmd.omega;
        cv = cmd.curv;
        s  = projectToPath({p.x, p.y}, s).s;

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

// --------------------------------------------------------------- control pose
mrpt::math::TPose2D TrajectoryFollower::controlPose(
    const VehicleLocalizationState& loc, const VehicleOdometryState& odo)
{
    // No odometry to propagate: track the raw localization directly.
    if (!odo.valid)
    {
        ctrlPoseInit_ = false;
        return loc.pose;
    }

    const mrpt::poses::CPose2D odoP(odo.odometry);

    if (!ctrlPoseInit_)
    {
        ctrlPose_     = mrpt::poses::CPose2D(loc.pose);
        lastOdom_     = odoP;
        ctrlPoseInit_ = true;
        return ctrlPose_.asTPose();
    }

    // Complementary filter of the control pose toward the raw localization:
    // wheel odometry is trusted for short-term motion, localization corrects
    // slowly. The filter state is the control pose itself, dead-reckoned with
    // per-cycle odometry increments, and every correction is applied AT the
    // vehicle. An earlier formulation filtered the map->odom anchor instead;
    // its partially-applied (low-passed/clamped) yaw corrections rotate the
    // composed pose about the odom ORIGIN, so their effect at the vehicle is
    // amplified by a lever arm equal to the distance driven since odometry
    // started -- the filter's effective bandwidth and its residual error
    // under localization yaw jitter degraded linearly over a long mission.
    // Parameterized at the vehicle, behavior is independent of that
    // distance. Three stages:
    //  1. First-order low-pass (anchor_time_constant): with a jittery
    //     localization the residual moves every cycle; the low-pass averages
    //     the jitter out instead of letting the control pose chase it. A
    //     pure slew limiter is the wrong filter for sustained jitter: once
    //     the residual moves faster than the slew rate, it saturates into a
    //     heavy nonlinear lag (the phase shift that turns noise into a
    //     closed-loop weave).
    //  2. Rate clamp (anchor_max_lin/ang_rate): bounds how fast a genuine
    //     one-shot relocalization jump is absorbed, so the commanded
    //     curvature never lurches.
    //  3. Divergence bound (anchor_max_lin/ang_divergence): hard cap on how
    //     far the filtered control pose may stray from the raw localization,
    //     so sustained heavy jitter can never leave the vehicle steering on
    //     a stale pose far from anywhere the localization believes it is.
    // Between relocalizations with a clean localization the residual is
    // zero, all three stages are no-ops, and the control pose equals the
    // localization exactly.

    // Dead-reckon with the odometry increment (a relative quantity: immune
    // to where the odom origin is).
    ctrlPose_ = ctrlPose_ + (odoP - lastOdom_);
    lastOdom_ = odoP;

    // Correction residual toward the raw localization, in the control pose's
    // own (vehicle) frame.
    const mrpt::math::TPose2D d =
        (mrpt::poses::CPose2D(loc.pose) - ctrlPose_).asTPose();
    const double dt    = params.control_period;
    const double alpha = params.anchor_time_constant > 0
                             ? 1.0 - std::exp(-dt / params.anchor_time_constant)
                             : 1.0;

    mrpt::math::TPose2D step;
    step.x   = d.x * alpha;
    step.y   = d.y * alpha;
    step.phi = mrpt::math::wrapToPi(d.phi) * alpha;

    const double maxLin = params.anchor_max_lin_rate * dt;
    const double maxAng = params.anchor_max_ang_rate * dt;
    const double lin    = std::hypot(step.x, step.y);
    if (lin > maxLin)
    {
        const double s = maxLin / lin;
        step.x         = step.x * s;
        step.y         = step.y * s;
    }
    step.phi  = std::clamp(step.phi, -maxAng, maxAng);
    ctrlPose_ = ctrlPose_ + mrpt::poses::CPose2D(step);

    // Divergence bounds, both direct now that the state lives at the
    // vehicle: excess yaw is absorbed in place (position untouched), excess
    // translation snapped along the residual (heading untouched).
    if (params.anchor_max_ang_divergence > 0.0)
    {
        const double dyaw =
            mrpt::math::wrapToPi(loc.pose.phi - ctrlPose_.phi());
        if (std::abs(dyaw) > params.anchor_max_ang_divergence)
        {
            const double e =
                dyaw - std::copysign(params.anchor_max_ang_divergence, dyaw);
            ctrlPose_ = mrpt::poses::CPose2D(
                ctrlPose_.x(), ctrlPose_.y(), ctrlPose_.phi() + e);
        }
    }
    {
        const double dx   = loc.pose.x - ctrlPose_.x();
        const double dy   = loc.pose.y - ctrlPose_.y();
        const double dpos = std::hypot(dx, dy);
        if (params.anchor_max_lin_divergence > 0.0 &&
            dpos > params.anchor_max_lin_divergence)
        {
            const double s = (dpos - params.anchor_max_lin_divergence) / dpos;
            ctrlPose_      = mrpt::poses::CPose2D(
                     ctrlPose_.x() + dx * s, ctrlPose_.y() + dy * s,
                     ctrlPose_.phi());
        }
    }

    return ctrlPose_.asTPose();
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

    // Short-term tracking runs on a smooth control pose derived from the
    // high-rate wheel odometry (the map-frame localization only anchors the
    // path, see controlPose). With an identity/static map->odom (e.g. the unit
    // tests) this equals loc.pose exactly.
    const mrpt::math::TPose2D ctrlPose = controlPose(loc, odo);

    const Projection proj = projectToPath({ctrlPose.x, ctrlPose.y}, lastS_);
    lastS_                = proj.s;
    // Latched once from the real localized projection; reused below for both
    // the safety forecast and the emitted command sampling loop instead of
    // being re-decided per predicted sample (see advanceGear).
    const double gear = advanceGear(proj.s);

    // Desired body heading at the goal: the reference's own final heading, not
    // the last segment's travel direction (they are opposite on a reverse
    // approach, where the robot backs into the goal pose).
    const double endHeading = traj_.back().pose.phi;

    out.arc_length_s    = proj.s;
    out.cross_track_err = proj.cross_track;
    out.heading_err     = mrpt::math::wrapToPi(ctrlPose.phi - endHeading);

    // Goal reached? Use the Euclidean distance to the final path point (the
    // arc-length projection saturates before the robot physically arrives).
    //
    // Both the goal check and the arrival latch below are additionally gated on
    // the *arc-length* remaining, not the Euclidean distance alone: a
    // differential-drive entry path loops back on itself (it backs the robot
    // into the goal pose), so its forward leg can pass within a goal tolerance
    // of the goal *position* while the robot is still far from the path end in
    // arc-length and pointing the wrong way. Euclidean proximity alone would
    // then report the goal reached (or latch the terminal stop) mid-path,
    // before the reverse maneuver that actually seats the final heading has
    // run. Requiring the projection to be near the path end disambiguates a
    // forward-leg near-pass from a true arrival. Near the real end the path is
    // essentially straight, so this never blocks a legitimate arrival (there
    // arc-length remaining and Euclidean distance to the goal agree).
    const mrpt::math::TPoint2D goalPt = pointAtArc(totalLength());
    const double               distToGoal =
        std::hypot(goalPt.x - ctrlPose.x, goalPt.y - ctrlPose.y);
    const bool nearPathEnd = totalLength() - proj.s <= params.arrival_radius;
    if (nearPathEnd && distToGoal <= params.goal_dist_tol &&
        std::abs(out.heading_err) <= params.goal_ang_tol)
    {
        // Latch: ReachedGoal is terminal until a new trajectory is set. The
        // caller typically hands the vehicle to another controller on this
        // status; if that controller then moves the robot off the completed
        // path, an unlatched follower would re-awaken on the stale
        // trajectory and fight it with full-speed commands (observed as a
        // sustained crop-wall collision on a live run).
        arrived_            = true;
        lastCommandedSpeed_ = 0;
        out.status          = FollowerStatus::ReachedGoal;
        return out;  // empty command => node stops
    }

    // Terminal stop latch: hold a stop once the robot, already within
    // `arrival_radius` of the goal (in both Euclidean distance and path
    // arc-length, see above), has passed its closest approach and would
    // otherwise start driving away. On a kinematically infeasible final pose
    // (an Ackermann robot on a differential-drive path whose tail rotates in
    // place) the exact heading cannot be seated; this parks the robot at its
    // closest approach instead of running away and thrashing into nearby
    // obstacles.
    if (nearPathEnd)
    {
        minDistToGoal_ = std::min(minDistToGoal_, distToGoal);
        if (!arrived_ && distToGoal <= params.arrival_radius &&
            distToGoal > minDistToGoal_ + 0.03)
            arrived_ = true;
    }
    if (arrived_)
    {
        // Settled at the closest approach the vehicle can reach. On a
        // differential-drive path an Ackermann robot often cannot seat the
        // exact terminal heading (the planner's tail rotates in place); rather
        // than hold `Running` forever -- which hangs the caller with the robot
        // parked and no resolution -- report the goal as reached (position
        // best-effort). Any residual heading is left to the downstream maneuver
        // (e.g. the reactive corridor-follower that backs into the row).
        lastCommandedSpeed_ = 0;
        out.status          = FollowerStatus::ReachedGoal;
        out.target_speed    = 0;
        return out;  // empty command => node stops
    }

    // Evaluate OffPathExceeded against the raw localized pose, not the
    // odometry-smoothed control pose. controlPose() rate-limits its correction
    // toward the localization to keep the wheel command smooth across
    // relocalization jumps, but that same slew makes the smoothed pose lag the
    // true localization while the correction is being absorbed -- judging "off
    // path" by it turns a normal localization correction into a phantom
    // tracking error even when the robot is physically on the path. The command still uses `proj` (ctrlPose) for
    // smoothness; only the fault test uses the true pose. A genuine sustained
    // deviation still trips it; a brief localization glitch is left for the
    // caller to debounce. In the identity/static map->odom case (unit tests)
    // ctrlPose == loc.pose, so this is a no-op there.
    const auto nowStamp =
        loc.timestamp != INVALID_TIMESTAMP ? loc.timestamp : mrpt::Clock::now();

    double offPathCross = proj.cross_track;
    if (loc.valid)
    {
        offPathCross =
            projectToPath({loc.pose.x, loc.pose.y}, proj.s).cross_track;
    }
    // Debounced fault: the raw-localization cross-track can spike
    // transiently (localization jitter), so OffPathExceeded only fires once
    // it has stayed over the limit for off_path_min_duration continuously.
    const bool offPathNow = std::abs(offPathCross) > params.max_cross_track;
    if (offPathNow)
    {
        if (offPathSince_ == INVALID_TIMESTAMP)
        {
            offPathSince_ = nowStamp;
        }
    }
    else
    {
        offPathSince_ = INVALID_TIMESTAMP;
    }
    const bool offPathLatched =
        offPathNow &&
        (params.off_path_min_duration <= 0.0 ||
         mrpt::system::timeDifference(offPathSince_, nowStamp) >=
             params.off_path_min_duration);
    out.status = offPathLatched ? FollowerStatus::OffPathExceeded
                                : FollowerStatus::Running;

    // Seed the speed ramp from the follower's own last commanded speed (a
    // feedforward integrator), not the measured odometry velocity: the profile
    // must keep accelerating even when the odometry source reports no forward
    // twist. Safety is handled by the predictive-safety scale and the node
    // watchdog, not by throttling the ramp to measured velocity.
    // Signed: reverse maneuvers ramp toward a negative speed.
    double predV = lastCommandedSpeed_;
    // Likewise seeds the omega/curvature rate limiters
    // (Parameters::max_omega_rate / max_curvature_rate) from the follower's
    // own last commanded values, mirroring predV above.
    double predOmega = lastCommandedOmega_;
    double predCurv  = lastCommandedCurv_;

    // Predictive safety: sweep the footprint over the command forecast and the
    // reference path ahead, and scale the commanded speed toward a stop before
    // contact. A hysteresis latch avoids chattering; a sustained stop reports
    // Blocked. Inert when no obstacles/footprint are set.
    double scale = 1.0;
    if (!obstacles_.empty())
    {
        const double dFwd = forecastContactDistance(
            ctrlPose, predV, predOmega, predCurv, proj.s, gear);
        const double dRef = referenceContactDistance(proj.s);
        scale             = std::min(
                        contactDistanceToScale(dFwd), contactDistanceToScale(dRef));
    }
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

    // Debug trace (off unless COutputLogger verbosity is raised to
    // LVL_DEBUG): one summary line per control cycle -- the actually
    // published/commanded state, as opposed to [pursuit]'s per-forecast-
    // sample internals above. `rosT` is loc.timestamp (the pose's own
    // stamp, i.e. the same UNIX-epoch convention as a ROS header.stamp when
    // use_sim_time is false) rather than log-print wall time, so lines can
    // be lined up against a recorded bag's topic timestamps directly.
    MRPT_LOG_DEBUG_STREAM(
        "[step] rosT=" << std::fixed << std::setprecision(6)
                       << mrpt::Clock::toDouble(nowStamp)
                       << " status=" << static_cast<int>(out.status)
                       << " s=" << proj.s << "/" << totalLength()
                       << " gear=" << gear << " crossTrack=" << proj.cross_track
                       << " offPathCross=" << offPathCross << " headingErr="
                       << out.heading_err << " distToGoal=" << distToGoal
                       << " safetyScale=" << scale << " stopped=" << stopped_);

    // map->odom correction so the emitted chunk is expressed in the odom frame.
    // Built from the same smooth control pose used for tracking, so the chunk's
    // first sample sits exactly at the current odometry pose and stays
    // continuous across a relocalization (the anchor slew, not the wheels,
    // absorbs the jump).
    const mrpt::poses::CPose2D map2odom =
        mrpt::poses::CPose2D(odo.odometry) +
        (mrpt::poses::CPose2D() - mrpt::poses::CPose2D(ctrlPose));

    mrpt::math::TPose2D predPose = ctrlPose;
    double              predS    = proj.s;

    out.command.frame_id = params.emit_frame;
    out.command.stamp    = loc.timestamp;

    const int nSamples = std::max(
        1, static_cast<int>(std::ceil(params.horizon / params.sample_period)));

    for (int k = 0; k <= nSamples; k++)
    {
        // The k==0 sample is the command actually executed until the next
        // control cycle, so its accel/omega/curvature rate-limit step must
        // use control_period; later samples are forecast at sample_period.
        // Otherwise, with control_period < sample_period, every executed
        // command would be allowed sample_period-sized steps at
        // control_period cadence -- all rate limits would effectively run at
        // (sample_period / control_period) times their configured rate.
        const double dtK = k == 0 ? params.control_period
                                  : params.sample_period;
        const Command cmd = pursuit(
            predPose, predV, predOmega, predCurv, predS, dtK, gear, scale);

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
            // Persist for the next cycle's feedforward ramp seed.
            lastCommandedSpeed_ = cmd.v;
            lastCommandedOmega_ = cmd.omega;
            lastCommandedCurv_  = cmd.curv;
        }

        // Advance the forecast.
        predPose =
            integrateUnicycle(predPose, cmd.v, cmd.omega, params.sample_period);
        predV     = cmd.v;
        predOmega = cmd.omega;
        predCurv  = cmd.curv;
        predS     = projectToPath({predPose.x, predPose.y}, predS).s;

        if (totalLength() - predS <= params.goal_dist_tol) break;
    }

    return out;
}
