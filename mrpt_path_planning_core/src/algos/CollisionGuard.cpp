/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CollisionGuard.h>
#include <mrpt/config/CConfigFileBase.h>  // MCP_LOAD_*
#include <mrpt/math/TPose2D.h>

#include <algorithm>
#include <cmath>
#include <variant>

using namespace mpp;

namespace
{
constexpr double kInf  = std::numeric_limits<double>::infinity();
constexpr double kMinV = 1e-3;  // [m/s]
constexpr double kMinW = 1e-3;  // [rad/s]

// Robot pose after traveling a signed arc length `sigma` along an arc of
// curvature `curv`, starting at the origin.
mrpt::math::TPose2D arcPose(double sigma, double curv)
{
    if (std::abs(curv) < 1e-9) { return {sigma, 0, 0}; }
    const double th = curv * sigma;
    return {std::sin(th) / curv, (1.0 - std::cos(th)) / curv, th};
}

// Stopping travel after a reaction time tR at speed v, then braking at a.
double stopTravel(double v, double tR, double a)
{
    v = std::abs(v);
    return v * tR + v * v / (2.0 * a);
}

// Largest speed whose stopping travel (see stopTravel) fits in d.
double maxSpeedForTravel(double d, double tR, double a)
{
    if (d <= 0) { return 0; }
    const double atR = a * tR;
    return -atR + std::sqrt(atR * atR + 2.0 * a * d);
}
}  // namespace

// ---------------------------------------------------------------- Parameters
CollisionGuard::Parameters CollisionGuard::Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    Parameters p;
    p.load_from_yaml(c);
    return p;
}

mrpt::containers::yaml CollisionGuard::Parameters::as_yaml() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();
    MCP_SAVE(c, max_decel);
    MCP_SAVE(c, max_ang_decel);
    MCP_SAVE(c, reaction_time);
    MCP_SAVE(c, margin);
    MCP_SAVE(c, max_obstacles_age);
    return c;
}

void CollisionGuard::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, max_decel);
    MCP_LOAD_OPT(c, max_ang_decel);
    MCP_LOAD_OPT(c, reaction_time);
    MCP_LOAD_OPT(c, margin);
    MCP_LOAD_OPT(c, max_obstacles_age);
    ASSERT_GT_(max_decel, 0.0);
    ASSERT_GT_(max_ang_decel, 0.0);
    ASSERT_GE_(reaction_time, 0.0);
    ASSERT_GE_(margin, 0.0);
}

// ------------------------------------------------------------------- inputs
void CollisionGuard::setRobotShape(const RobotShape& shape)
{
    poly_.clear();
    radius_    = 0;
    maxRadius_ = 0;
    if (const auto* poly = std::get_if<mrpt::math::TPolygon2D>(&shape))
    {
        poly_.assign(poly->begin(), poly->end());
        for (const auto& v : poly_)
        {
            maxRadius_ = std::max(maxRadius_, v.norm());
        }
    }
    else if (const auto* r = std::get_if<robot_radius_t>(&shape))
    {
        radius_    = *r;
        maxRadius_ = *r;
    }
}

void CollisionGuard::setObstacles(
    const std::vector<mrpt::math::TPoint2D>& pts,
    mrpt::system::TTimeStamp                 stamp)
{
    obstacles_      = pts;
    obstaclesStamp_ = stamp;
}

void CollisionGuard::setObstacles(
    const mrpt::maps::CPointsMap& pts, mrpt::system::TTimeStamp stamp)
{
    const auto& xs = pts.getPointsBufferRef_x();
    const auto& ys = pts.getPointsBufferRef_y();
    obstacles_.resize(xs.size());
    for (size_t i = 0; i < xs.size(); i++) { obstacles_[i] = {xs[i], ys[i]}; }
    obstaclesStamp_ = stamp;
}

// ----------------------------------------------------------------- geometry
double CollisionGuard::signedDistance(const mrpt::math::TPoint2D& p) const
{
    if (poly_.size() < 3) { return p.norm() - radius_; }

    bool   inside = false;
    double minD2  = kInf;
    for (size_t i = 0, j = poly_.size() - 1; i < poly_.size(); j = i++)
    {
        const auto& a = poly_[j];
        const auto& b = poly_[i];
        // Crossing-number inside test:
        if (((b.y > p.y) != (a.y > p.y)) &&
            (p.x < (a.x - b.x) * (p.y - b.y) / (a.y - b.y) + b.x))
        {
            inside = !inside;
        }
        // Distance to segment [a,b]:
        const double ex = b.x - a.x;
        const double ey = b.y - a.y;
        const double l2 = ex * ex + ey * ey;
        double t = l2 > 0 ? ((p.x - a.x) * ex + (p.y - a.y) * ey) / l2 : 0;
        t        = std::clamp(t, 0.0, 1.0);
        const double dx = p.x - (a.x + t * ex);
        const double dy = p.y - (a.y + t * ey);
        minD2           = std::min(minD2, dx * dx + dy * dy);
    }
    const double d = std::sqrt(minD2);
    return inside ? -d : d;
}

template <typename POSE_AT>
double CollisionGuard::sweep(
    POSE_AT poseAt, double dispPerUnit, double maxT, double reach,
    bool* inContact, std::optional<mrpt::math::TPoint2D>* contactPoint) const
{
    // Only obstacles the footprint could reach:
    const double reachR  = reach + maxRadius_ + params.margin;
    const double reachR2 = reachR * reachR;

    std::vector<mrpt::math::TPoint2D> candidates;
    bool                              contact      = false;
    double                            clearanceNow = kInf;
    for (const auto& p : obstacles_)
    {
        if (p.sqrNorm() > reachR2) { continue; }
        const double d0 = signedDistance(p);
        if (d0 <= 0)
        {
            contact = true;
            continue;
        }
        clearanceNow = std::min(clearanceNow, d0);
        candidates.push_back(p);
    }
    // If already within the margin, only forbid getting even closer:
    const double thr = std::min(params.margin, 0.5 * clearanceNow);
    if (inContact) { *inContact = contact; }
    if (candidates.empty() || maxT <= 0) { return kInf; }

    // Step such that no footprint point moves more than thr/2 between
    // samples, so a point cannot cross the threshold unnoticed (with a floor
    // of 1 mm of displacement, to bound the cost for a near-zero threshold):
    constexpr double kMinDisp = 1e-3;  // [m]
    const double     dt =
        std::max(kMinDisp, 0.5 * thr) / std::max(dispPerUnit, 1e-6);

    for (double t = dt;; t += dt)
    {
        t                 = std::min(t, maxT);
        const auto   pose = poseAt(t);
        const double c    = std::cos(pose.phi);
        const double s    = std::sin(pose.phi);
        const double rr   = maxRadius_ + thr;
        for (const auto& p : candidates)
        {
            // Obstacle in the robot frame at this pose:
            const double               dx = p.x - pose.x;
            const double               dy = p.y - pose.y;
            const mrpt::math::TPoint2D q(c * dx + s * dy, -s * dx + c * dy);
            if (q.sqrNorm() > rr * rr) { continue; }
            if (signedDistance(q) <= thr)
            {
                if (contactPoint) { *contactPoint = p; }
                return std::max(0.0, t - dt);
            }
        }
        if (t >= maxT) { break; }
    }
    return kInf;
}

double CollisionGuard::freeDistance(
    double v, double omega, double maxDist, bool* inContact,
    std::optional<mrpt::math::TPoint2D>* contactPoint) const
{
    ASSERT_(v != 0);
    const double sgn  = v > 0 ? 1.0 : -1.0;
    const double curv = omega / v;
    // Max displacement of any footprint point per unit of arc length:
    const double dispPerUnit = 1.0 + std::abs(curv) * maxRadius_;
    return sweep(
        [&](double s) { return arcPose(sgn * s, curv); }, dispPerUnit, maxDist,
        maxDist, inContact, contactPoint);
}

double CollisionGuard::freeAngle(
    double omega, double maxAngle, bool* inContact,
    std::optional<mrpt::math::TPoint2D>* contactPoint) const
{
    ASSERT_(omega != 0);
    const double sgn = omega > 0 ? 1.0 : -1.0;
    // Max displacement of any footprint point per radian:
    return sweep(
        [&](double a) { return mrpt::math::TPose2D(0, 0, sgn * a); },
        std::max(maxRadius_, 1e-3), maxAngle, 0.0, inContact, contactPoint);
}

// ------------------------------------------------------------------- filter
CollisionGuard::Motion CollisionGuard::checkMotion(
    double v, double omega, double vStop, double tR) const
{
    Motion m;
    if (std::abs(v) >= kMinV)
    {
        // Translation along an arc. When braking at constant curvature, the
        // angular deceleration is curv times the linear one, so the latter is
        // also bounded by the angular limit:
        m.curv = omega / v;
        m.a    = params.max_decel;
        if (std::abs(m.curv) > 1e-6)
        {
            m.a = std::min(m.a, params.max_ang_decel / std::abs(m.curv));
        }
        m.need = stopTravel(vStop, tR, m.a);
        m.free = freeDistance(
            v, omega, m.need + 0.05, &m.inContact, &m.contactPoint);
    }
    else
    {
        // Rotation in place:
        m.rotation = true;
        m.a        = params.max_ang_decel;
        m.need     = stopTravel(vStop, tR, m.a);
        m.free = freeAngle(omega, m.need + 0.02, &m.inContact, &m.contactPoint);
    }
    return m;
}

CollisionGuard::Result CollisionGuard::filter(
    double v, double omega, mrpt::system::TTimeStamp now,
    const std::optional<mrpt::math::TTwist2D>& currentVel) const
{
    Result r;
    r.v     = v;
    r.omega = omega;

    auto fullStop = [&r]()
    {
        r.v       = 0;
        r.omega   = 0;
        r.limited = true;
    };

    // Stopping is always safe:
    if (std::abs(v) < kMinV && std::abs(omega) < kMinW) { return r; }

    // Obstacle data age (fail-safe: no data, or too old => stop):
    double age = 0;
    if (obstaclesStamp_ == INVALID_TIMESTAMP)
    {
        // Never received any obstacle data: always fail safe.
        r.stale = true;
    }
    else
    {
        age = std::max(0.0, mrpt::system::timeDifference(obstaclesStamp_, now));
        r.stale =
            params.max_obstacles_age > 0 && age > params.max_obstacles_age;
    }
    if (r.stale)
    {
        fullStop();
        return r;
    }

    const double tR = params.reaction_time + age;

    // Is the current motion itself safe? If not, stop right now:
    double vCur = 0;
    double wCur = 0;
    if (currentVel)
    {
        vCur = currentVel->vx;
        wCur = currentVel->omega;
        if (std::abs(vCur) >= kMinV || std::abs(wCur) >= kMinW)
        {
            const bool isRot = std::abs(vCur) < kMinV;
            const auto cur   = checkMotion(vCur, wCur, isRot ? wCur : vCur, tR);
            r.in_contact     = cur.inContact;
            if (cur.free < cur.need)
            {
                r.free_travel           = cur.free;
                r.limiting_point        = cur.contactPoint;
                r.current_motion_unsafe = true;
                fullStop();
                return r;
            }
        }
    }

    // The command, using the largest of the commanded and current speeds to
    // evaluate the stopping travel:
    const bool   isRot = std::abs(v) < kMinV;
    const double vStop = isRot ? std::max(std::abs(omega), std::abs(wCur))
                               : std::max(std::abs(v), std::abs(vCur));
    const auto   cmd   = checkMotion(v, omega, vStop, tR);
    r.in_contact       = r.in_contact || cmd.inContact;
    r.free_travel      = cmd.free;
    if (cmd.free >= cmd.need) { return r; }
    r.limiting_point = cmd.contactPoint;

    // Reduce the speed, keeping the arc (or the rotation direction):
    const double cmdMag  = isRot ? std::abs(omega) : std::abs(v);
    const double curMag  = isRot ? std::abs(wCur) : std::abs(vCur);
    const double safeMag = maxSpeedForTravel(cmd.free, tR, cmd.a);
    r.limited            = true;
    if (safeMag < (isRot ? kMinW : kMinV) || safeMag < curMag)
    {
        // Not even the current speed is safe on this arc:
        fullStop();
        return r;
    }
    const double mag = std::min(cmdMag, safeMag);
    if (isRot)
    {
        r.v     = 0;
        r.omega = (omega > 0 ? 1.0 : -1.0) * mag;
    }
    else
    {
        r.v     = (v > 0 ? 1.0 : -1.0) * mag;
        r.omega = r.v * cmd.curv;
    }
    return r;
}
