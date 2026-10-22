/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorTurning.h>
#include <mrpt/config/CConfigFileBase.h>  // MCP_LOAD_*
#include <mrpt/math/wrap2pi.h>

#include <algorithm>
#include <cmath>

IMPLEMENTS_MRPT_OBJECT(CostEvaluatorTurning, CostEvaluator, mpp)

using namespace mpp;

CostEvaluatorTurning::Parameters CostEvaluatorTurning::Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    Parameters p;
    p.load_from_yaml(c);
    return p;
}

mrpt::containers::yaml CostEvaluatorTurning::Parameters::as_yaml() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();
    MCP_SAVE(c, turnCostFactor);
    return c;
}

void CostEvaluatorTurning::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());
    MCP_LOAD_OPT(c, turnCostFactor);
    ASSERT_(std::isfinite(turnCostFactor));
    ASSERT_GE_(turnCostFactor, 0.0);
}

void CostEvaluatorTurning::setPTGs(const TrajectoriesAndRobotShape& ptgs)
{
    ptgs_ = ptgs.ptgs;
}

double CostEvaluatorTurning::headingChange(
    int ptgIndex, int pathIndex, uint32_t step) const
{
    if (ptgIndex < 0 || pathIndex < 0 ||
        static_cast<size_t>(ptgIndex) >= ptgs_.size())
    {
        return 0;
    }
    const auto& ptg = ptgs_[ptgIndex];
    if (!ptg || static_cast<size_t>(pathIndex) >= ptg->getPathCount())
    {
        return 0;
    }
    const uint32_t nSteps = std::min<uint32_t>(
        step, ptg->getPathStepCount(static_cast<uint16_t>(pathIndex)) - 1);

    // Sampled finely enough for unambiguous heading increments even on the
    // tightest loops:
    constexpr uint32_t kSamples = 16;
    const uint32_t     stride   = std::max<uint32_t>(1, nSteps / kSamples);

    double   total = 0;
    double   prev  = ptg->getPathPose(pathIndex, 0).phi;
    uint32_t s     = 0;
    while (s < nSteps)
    {
        s                = std::min(s + stride, nSteps);
        const double phi = ptg->getPathPose(pathIndex, s).phi;
        total += std::abs(mrpt::math::wrapToPi(phi - prev));
        prev = phi;
    }
    return total;
}

bool CostEvaluatorTurning::isCusp(const MoveEdgeSE2_TPS& edge) const
{
    if (edge.ptgIndex < 0 || edge.ptgPathIndex < 0 ||
        static_cast<size_t>(edge.ptgIndex) >= ptgs_.size())
    {
        return false;
    }
    const auto& ptg = ptgs_[edge.ptgIndex];
    if (!ptg || static_cast<size_t>(edge.ptgPathIndex) >= ptg->getPathCount())
    {
        return false;
    }
    // Speed along the heading at the edge start (global velocity):
    constexpr double kMinSpeed = 0.05;  // [m/s]
    const auto&      st        = edge.stateFrom;
    const double     v =
        st.vel.vx * std::cos(st.pose.phi) + st.vel.vy * std::sin(st.pose.phi);
    if (std::abs(v) < kMinSpeed) { return false; }
    const double edgeV = ptg->getPathTwist(edge.ptgPathIndex, 0).vx;
    return v * edgeV < 0;
}

bool CostEvaluatorTurning::stopsMovingRobot(const MoveEdgeSE2_TPS& edge) const
{
    if (isCusp(edge)) { return true; }
    if (edge.ptgIndex < 0 || edge.ptgPathIndex < 0 ||
        static_cast<size_t>(edge.ptgIndex) >= ptgs_.size())
    {
        return false;
    }
    const auto& ptg = ptgs_[edge.ptgIndex];
    if (!ptg || static_cast<size_t>(edge.ptgPathIndex) >= ptg->getPathCount())
    {
        return false;
    }
    constexpr double kMinSpeed = 0.05;  // [m/s]
    const auto&      st        = edge.stateFrom;
    if (std::hypot(st.vel.vx, st.vel.vy) < kMinSpeed) { return false; }
    // A rotation in place:
    const auto tw = ptg->getPathTwist(edge.ptgPathIndex, 0);
    return std::hypot(tw.vx, tw.vy) < 1e-6 && std::abs(tw.omega) > 1e-6;
}

double CostEvaluatorTurning::operator()(const MoveEdgeSE2_TPS& edge) const
{
    if (params_.turnCostFactor <= 0) { return 0; }
    // A cusp turns the direction of travel by pi, without changing the
    // heading, and rotating in place also needs a stop: otherwise stopping
    // would be cheaper than turning while driving.
    return params_.turnCostFactor *
           (headingChange(edge.ptgIndex, edge.ptgPathIndex, edge.ptgStepIndex) +
            (stopsMovingRobot(edge) ? M_PI : 0.0));
}
