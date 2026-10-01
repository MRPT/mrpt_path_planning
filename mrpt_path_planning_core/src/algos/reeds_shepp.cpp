/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/* The path formulas and path-type table below are adapted from OMPL's
 * ReedsSheppStateSpace.cpp:
 *
 *  Software License Agreement (BSD License)
 *  Copyright (c) 2010, Rice University. All rights reserved.
 *  Author: Mark Moll
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met: redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer;
 *  redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution; neither the name of
 *  the Rice University nor the names of its contributors may be used to
 *  endorse or promote products derived from this software without specific
 *  prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 *  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <mpp/algos/reeds_shepp.h>

#include <array>
#include <cmath>
#include <limits>

namespace
{
// Nomenclature of the Reeds-Shepp paper: (x, y, phi) is the goal pose in the
// frame of the start pose, scaled to unit turning radius. (t, u, v) are the
// signed lengths of the path segments (negative = reverse).

constexpr double kPi    = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kZero  = 10 * std::numeric_limits<double>::epsilon();

double mod2pi(double x)
{
    double v = std::fmod(x, kTwoPi);
    if (v < -kPi) { v += kTwoPi; }
    else if (v > kPi) { v -= kTwoPi; }
    return v;
}

void polar(double x, double y, double& r, double& theta)
{
    r     = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
}

void tauOmega(
    double u, double v, double xi, double eta, double phi, double& tau,
    double& omega)
{
    const double delta = mod2pi(u - v);
    const double A     = std::sin(u) - std::sin(delta);
    const double B     = std::cos(u) - std::cos(delta) - 1.;
    const double t1    = std::atan2(eta * A - xi * B, xi * A + eta * B);
    const double t2    = 2. * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3;
    tau                = (t2 < 0) ? mod2pi(t1 + kPi) : mod2pi(t1);
    omega              = mod2pi(tau - u + v - phi);
}

// formula 8.1
bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v)
{
    polar(x - std::sin(phi), y - 1. + std::cos(phi), u, t);
    if (t >= -kZero)
    {
        v = mod2pi(phi - t);
        if (v >= -kZero) { return true; }
    }
    return false;
}

// formula 8.2
bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v)
{
    double t1 = 0;
    double u1 = 0;
    polar(x + std::sin(phi), y - 1. - std::cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.)
    {
        u                  = std::sqrt(u1 - 4.);
        const double theta = std::atan2(2., u);
        t                  = mod2pi(t1 + theta);
        v                  = mod2pi(t - phi);
        return t >= -kZero && v >= -kZero;
    }
    return false;
}

// formula 8.3 / 8.4
bool LpRmL(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi    = x - std::sin(phi);
    const double eta   = y - 1. + std::cos(phi);
    double       u1    = 0;
    double       theta = 0;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.)
    {
        u = -2. * std::asin(.25 * u1);
        t = mod2pi(theta + .5 * u + kPi);
        v = mod2pi(phi - t + u);
        return t >= -kZero && u <= kZero;
    }
    return false;
}

// formula 8.7
bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi  = x + std::sin(phi);
    const double eta = y - 1. - std::cos(phi);
    const double rho = .25 * (2. + std::sqrt(xi * xi + eta * eta));
    if (rho <= 1.)
    {
        u = std::acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        return t >= -kZero && v <= kZero;
    }
    return false;
}

// formula 8.8
bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi  = x + std::sin(phi);
    const double eta = y - 1. - std::cos(phi);
    const double rho = (20. - xi * xi - eta * eta) / 16.;
    if (rho >= 0 && rho <= 1)
    {
        u = -std::acos(rho);
        if (u >= -.5 * kPi)
        {
            tauOmega(u, u, xi, eta, phi, t, v);
            return t >= -kZero && v >= -kZero;
        }
    }
    return false;
}

// formula 8.9
bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi    = x - std::sin(phi);
    const double eta   = y - 1. + std::cos(phi);
    double       rho   = 0;
    double       theta = 0;
    polar(xi, eta, rho, theta);
    if (rho >= 2.)
    {
        const double r = std::sqrt(rho * rho - 4.);
        u              = 2. - r;
        t              = mod2pi(theta + std::atan2(r, -2.));
        v              = mod2pi(phi - .5 * kPi - t);
        return t >= -kZero && u <= kZero && v <= kZero;
    }
    return false;
}

// formula 8.10
bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi    = x + std::sin(phi);
    const double eta   = y - 1. - std::cos(phi);
    double       rho   = 0;
    double       theta = 0;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.)
    {
        t = theta;
        u = 2. - rho;
        v = mod2pi(t + .5 * kPi - phi);
        return t >= -kZero && u <= kZero && v <= kZero;
    }
    return false;
}

// formula 8.11
bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v)
{
    const double xi    = x + std::sin(phi);
    const double eta   = y - 1. - std::cos(phi);
    double       rho   = 0;
    double       theta = 0;
    polar(xi, eta, rho, theta);
    if (rho >= 2.)
    {
        u = 4. - std::sqrt(rho * rho - 4.);
        if (u <= kZero)
        {
            t = mod2pi(
                std::atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
            v = mod2pi(t - phi);
            return t >= -kZero && v >= -kZero;
        }
    }
    return false;
}

using word_fn_t = bool (*)(double, double, double, double&, double&, double&);

// Evaluates one path word under the four symmetries (identity, timeflip,
// reflect, timeflip + reflect), keeping the shortest total length. The word
// length is |t| + uWeight*|u| + |v| + fixedLength.
void tryWord(
    word_fn_t fn, double x, double y, double phi, double uWeight,
    double fixedLength, double& best)
{
    double                      t    = 0;
    double                      u    = 0;
    double                      v    = 0;
    const std::array<double, 4> xs   = {x, -x, x, -x};
    const std::array<double, 4> ys   = {y, y, -y, -y};
    const std::array<double, 4> phis = {phi, -phi, -phi, phi};
    for (size_t i = 0; i < xs.size(); i++)
    {
        if (fn(xs[i], ys[i], phis[i], t, u, v))
        {
            const double L =
                std::abs(t) + uWeight * std::abs(u) + std::abs(v) + fixedLength;
            if (L < best) { best = L; }
        }
    }
}

double shortestUnitLength(double x, double y, double phi)
{
    double best = std::numeric_limits<double>::max();

    // Coordinates for the "backwards" words (path traversed in reverse):
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);

    // CSC
    tryWord(&LpSpLp, x, y, phi, 1.0, 0.0, best);
    tryWord(&LpSpRp, x, y, phi, 1.0, 0.0, best);
    // CCC
    tryWord(&LpRmL, x, y, phi, 1.0, 0.0, best);
    tryWord(&LpRmL, xb, yb, phi, 1.0, 0.0, best);
    // CCCC
    tryWord(&LpRupLumRm, x, y, phi, 2.0, 0.0, best);
    tryWord(&LpRumLumRp, x, y, phi, 2.0, 0.0, best);
    // CCSC (one fixed quarter-turn arc)
    tryWord(&LpRmSmLm, x, y, phi, 1.0, .5 * kPi, best);
    tryWord(&LpRmSmRm, x, y, phi, 1.0, .5 * kPi, best);
    tryWord(&LpRmSmLm, xb, yb, phi, 1.0, .5 * kPi, best);
    tryWord(&LpRmSmRm, xb, yb, phi, 1.0, .5 * kPi, best);
    // CCSCC (two fixed quarter-turn arcs)
    tryWord(&LpRmSLmRp, x, y, phi, 1.0, kPi, best);

    return best;
}
// Segment types of each Reeds-Shepp path family ('N': unused slot), as in
// OMPL's reedsSheppPathType table.
constexpr std::array<std::array<char, 5>, 18> kPathTypes = {{
    {'L', 'R', 'L', 'N', 'N'},  // 0
    {'R', 'L', 'R', 'N', 'N'},  // 1
    {'L', 'R', 'L', 'R', 'N'},  // 2
    {'R', 'L', 'R', 'L', 'N'},  // 3
    {'L', 'R', 'S', 'L', 'N'},  // 4
    {'R', 'L', 'S', 'R', 'N'},  // 5
    {'L', 'S', 'R', 'L', 'N'},  // 6
    {'R', 'S', 'L', 'R', 'N'},  // 7
    {'L', 'R', 'S', 'R', 'N'},  // 8
    {'R', 'L', 'S', 'L', 'N'},  // 9
    {'R', 'S', 'R', 'L', 'N'},  // 10
    {'L', 'S', 'L', 'R', 'N'},  // 11
    {'L', 'S', 'R', 'N', 'N'},  // 12
    {'R', 'S', 'L', 'N', 'N'},  // 13
    {'L', 'S', 'L', 'N', 'N'},  // 14
    {'R', 'S', 'R', 'N', 'N'},  // 15
    {'L', 'R', 'S', 'L', 'R'},  // 16
    {'R', 'L', 'S', 'R', 'L'}  // 17
}};

// Shortest path so far, in unit-radius lengths.
struct UnitPath
{
    int                   type  = -1;
    std::array<double, 5> len   = {0, 0, 0, 0, 0};
    double                total = std::numeric_limits<double>::max();
};

void keepIfShorter(UnitPath& best, int type, const std::array<double, 5>& len)
{
    double total = 0;
    for (const double l : len) { total += std::abs(l); }
    if (total < best.total)
    {
        best.type  = type;
        best.len   = len;
        best.total = total;
    }
}

using lengths_t = std::array<double, 5>;
using make_fn_t = lengths_t (*)(double, double, double);

// Tries one path word on (x, y, phi); on success, maps its (t, u, v) to the
// segment lengths of path type `type`.
void tryPath(
    word_fn_t fn, double x, double y, double phi, int type, make_fn_t make,
    UnitPath& best)
{
    double t = 0;
    double u = 0;
    double v = 0;
    if (fn(x, y, phi, t, u, v)) { keepIfShorter(best, type, make(t, u, v)); }
}

// Each word is tried under the four symmetries: identity (x, y, phi),
// timeflip (-x, y, -phi) which negates the lengths, reflect (x, -y, -phi)
// which swaps left and right (the next table entry), and both (-x, -y, phi).
// `make` and `makeFlip` map (t, u, v) to lengths without and with timeflip.
void tryWordPaths(
    word_fn_t fn, double x, double y, double phi, int type, make_fn_t make,
    make_fn_t makeFlip, UnitPath& best)
{
    tryPath(fn, x, y, phi, type, make, best);
    tryPath(fn, -x, y, -phi, type, makeFlip, best);
    tryPath(fn, x, -y, -phi, type + 1, make, best);
    tryPath(fn, -x, -y, phi, type + 1, makeFlip, best);
}

constexpr double kHalfPi = .5 * kPi;

lengths_t csc(double t, double u, double v) { return {t, u, v, 0, 0}; }
lengths_t cscFlip(double t, double u, double v) { return {-t, -u, -v, 0, 0}; }
lengths_t cccBack(double t, double u, double v) { return {v, u, t, 0, 0}; }
lengths_t cccBackFlip(double t, double u, double v)
{
    return {-v, -u, -t, 0, 0};
}
lengths_t ccccUm(double t, double u, double v) { return {t, u, -u, v, 0}; }
lengths_t ccccUmFlip(double t, double u, double v)
{
    return {-t, -u, u, -v, 0};
}
lengths_t ccccUu(double t, double u, double v) { return {t, u, u, v, 0}; }
lengths_t ccccUuFlip(double t, double u, double v)
{
    return {-t, -u, -u, -v, 0};
}
lengths_t ccsc(double t, double u, double v) { return {t, -kHalfPi, u, v, 0}; }
lengths_t ccscFlip(double t, double u, double v)
{
    return {-t, kHalfPi, -u, -v, 0};
}
lengths_t ccscBack(double t, double u, double v)
{
    return {v, u, -kHalfPi, t, 0};
}
lengths_t ccscBackFlip(double t, double u, double v)
{
    return {-v, -u, kHalfPi, -t, 0};
}
lengths_t ccscc(double t, double u, double v)
{
    return {t, -kHalfPi, u, -kHalfPi, v};
}
lengths_t ccsccFlip(double t, double u, double v)
{
    return {-t, kHalfPi, -u, kHalfPi, -v};
}

UnitPath shortestUnitPath(double x, double y, double phi)
{
    UnitPath best;
    // Coordinates for the "backwards" words (path traversed in reverse):
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);

    // CSC
    tryWordPaths(&LpSpLp, x, y, phi, 14, &csc, &cscFlip, best);
    tryWordPaths(&LpSpRp, x, y, phi, 12, &csc, &cscFlip, best);
    // CCC
    tryWordPaths(&LpRmL, x, y, phi, 0, &csc, &cscFlip, best);
    tryWordPaths(&LpRmL, xb, yb, phi, 0, &cccBack, &cccBackFlip, best);
    // CCCC
    tryWordPaths(&LpRupLumRm, x, y, phi, 2, &ccccUm, &ccccUmFlip, best);
    tryWordPaths(&LpRumLumRp, x, y, phi, 2, &ccccUu, &ccccUuFlip, best);
    // CCSC
    tryWordPaths(&LpRmSmLm, x, y, phi, 4, &ccsc, &ccscFlip, best);
    tryWordPaths(&LpRmSmRm, x, y, phi, 8, &ccsc, &ccscFlip, best);
    tryWordPaths(&LpRmSmLm, xb, yb, phi, 6, &ccscBack, &ccscBackFlip, best);
    tryWordPaths(&LpRmSmRm, xb, yb, phi, 10, &ccscBack, &ccscBackFlip, best);
    // CCSCC
    tryWordPaths(&LpRmSLmRp, x, y, phi, 16, &ccscc, &ccsccFlip, best);
    return best;
}

std::vector<mpp::ReedsSheppSegment> toSegments(
    const UnitPath& path, double turningRadius)
{
    std::vector<mpp::ReedsSheppSegment> out;
    if (path.type < 0) { return out; }
    for (size_t i = 0; i < 5; i++)
    {
        const char type = kPathTypes[path.type][i];
        if (type == 'N') { break; }
        if (std::abs(path.len[i]) <= kZero) { continue; }
        out.push_back({type, path.len[i] * turningRadius});
    }
    return out;
}
}  // namespace

double mpp::reeds_shepp_distance(
    const mrpt::math::TPose2D& from, const mrpt::math::TPose2D& to,
    double turningRadius)
{
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double c  = std::cos(from.phi);
    const double s  = std::sin(from.phi);
    const double x  = c * dx + s * dy;
    const double y  = -s * dx + c * dy;
    const double r  = turningRadius;
    return r * shortestUnitLength(x / r, y / r, to.phi - from.phi);
}

std::vector<mpp::ReedsSheppSegment> mpp::reeds_shepp_path(
    const mrpt::math::TPose2D& from, const mrpt::math::TPose2D& to,
    double turningRadius)
{
    const double dx  = to.x - from.x;
    const double dy  = to.y - from.y;
    const double c   = std::cos(from.phi);
    const double s   = std::sin(from.phi);
    const double r   = turningRadius;
    const double x   = (c * dx + s * dy) / r;
    const double y   = (-s * dx + c * dy) / r;
    const double phi = to.phi - from.phi;

    return toSegments(shortestUnitPath(x, y, phi), r);
}

std::vector<mpp::ReedsSheppSegment> mpp::reeds_shepp_path_to_point(
    const mrpt::math::TPose2D& from, const mrpt::math::TPoint2D& to,
    double turningRadius)
{
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double c  = std::cos(from.phi);
    const double s  = std::sin(from.phi);
    const double r  = turningRadius;
    const double x  = (c * dx + s * dy) / r;
    const double y  = (-s * dx + c * dy) / r;
    if (std::hypot(x, y) <= kZero) { return {}; }

    // Minimize the length over the final (relative) heading phi:
    double bestPhi = 0;
    double bestLen = std::numeric_limits<double>::max();
    auto   tryPhi  = [&](double phi)
    {
        const double len = shortestUnitLength(x, y, phi);
        if (len < bestLen)
        {
            bestLen = len;
            bestPhi = phi;
        }
        return len;
    };

    // Arc-then-straight paths: the final heading is tangent to the turning
    // circle centered at (0, side), forward or reverse, if the point lies
    // outside that circle. If it lies outside both, the best of these is used:
    // in a dense brute-force comparison it is the shortest path in most cases
    // and at most 0.07 turning radii longer otherwise.
    bool insideCircle = false;
    for (const double side : {1.0, -1.0})
    {
        const double cx = x;
        const double cy = y - side;
        const double d  = std::hypot(cx, cy);
        if (d < 1.0)
        {
            insideCircle = true;
            continue;
        }
        const double beta = std::atan2(cy, cx);
        const double a    = std::asin(1.0 / d);
        tryPhi(beta + side * a);
        tryPhi(beta + side * (kPi - a));
    }

    // Close to the start, inside a turning circle, the shortest paths have
    // other types (e.g., with a cusp): uniform sampling of the heading, then
    // golden-section refinement around the best one found.
    if (insideCircle)
    {
        constexpr int kSamples = 32;
        const double  step     = kTwoPi / kSamples;
        for (int i = 0; i < kSamples; i++) { tryPhi(i * step); }

        constexpr double kInvPhi = 0.61803398874989484820;
        double           lo      = bestPhi - step;
        double           hi      = bestPhi + step;
        double           m1      = hi - kInvPhi * (hi - lo);
        double           m2      = lo + kInvPhi * (hi - lo);
        double           f1      = tryPhi(m1);
        double           f2      = tryPhi(m2);
        for (int it = 0; it < 20; it++)
        {
            if (f1 < f2)
            {
                hi = m2;
                m2 = m1;
                f2 = f1;
                m1 = hi - kInvPhi * (hi - lo);
                f1 = tryPhi(m1);
            }
            else
            {
                lo = m1;
                m1 = m2;
                f1 = f2;
                m2 = lo + kInvPhi * (hi - lo);
                f2 = tryPhi(m2);
            }
        }
    }

    return toSegments(shortestUnitPath(x, y, bestPhi), r);
}

mrpt::math::TPose2D mpp::reeds_shepp_apply(
    const mrpt::math::TPose2D&            from,
    const std::vector<ReedsSheppSegment>& segments, double turningRadius)
{
    const double        r = turningRadius;
    mrpt::math::TPose2D p = from;
    for (const auto& seg : segments)
    {
        const double v = seg.length / r;  // unit-radius signed length
        if (seg.type == 'L')
        {
            p.x += r * (std::sin(p.phi + v) - std::sin(p.phi));
            p.y += r * (-std::cos(p.phi + v) + std::cos(p.phi));
            p.phi += v;
        }
        else if (seg.type == 'R')
        {
            p.x += r * (-std::sin(p.phi - v) + std::sin(p.phi));
            p.y += r * (std::cos(p.phi - v) - std::cos(p.phi));
            p.phi -= v;
        }
        else
        {
            p.x += seg.length * std::cos(p.phi);
            p.y += seg.length * std::sin(p.phi);
        }
    }
    p.normalizePhi();
    return p;
}
