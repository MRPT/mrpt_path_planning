/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/* The path formulas below are adapted from OMPL's ReedsSheppStateSpace.cpp,
 * keeping only the computation of the shortest path length:
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

constexpr double kPi    = M_PI;
constexpr double kTwoPi = 2.0 * M_PI;
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
