/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorReverseMotion.h>
#include <mrpt/config/CConfigFileBase.h>  // MCP_LOAD_*

IMPLEMENTS_MRPT_OBJECT(CostEvaluatorReverseMotion, CostEvaluator, mpp)

using namespace mpp;

CostEvaluatorReverseMotion::Parameters
    CostEvaluatorReverseMotion::Parameters::FromYAML(
        const mrpt::containers::yaml& c)
{
    Parameters p;
    p.load_from_yaml(c);
    return p;
}

mrpt::containers::yaml CostEvaluatorReverseMotion::Parameters::as_yaml() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();
    MCP_SAVE(c, reverseTimeCostFactor);
    return c;
}

void CostEvaluatorReverseMotion::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());
    MCP_LOAD_OPT(c, reverseTimeCostFactor);
    ASSERT_GE_(reverseTimeCostFactor, 0.0);
}

void CostEvaluatorReverseMotion::setPTGs(const TrajectoriesAndRobotShape& ptgs)
{
    isReverse_.clear();
    for (const auto& ptg : ptgs.ptgs)
    {
        auto& v = isReverse_.emplace_back();
        if (!ptg) { continue; }
        const auto nPaths = ptg->getPathCount();
        v.resize(nPaths);
        for (size_t k = 0; k < nPaths; k++)
        {
            v[k] = ptg->getPathTwist(k, 0).vx < 0;
        }
    }
}

bool CostEvaluatorReverseMotion::isReverse(int ptgIndex, int pathIndex) const
{
    if (ptgIndex < 0 || pathIndex < 0 ||
        static_cast<size_t>(ptgIndex) >= isReverse_.size())
    {
        return false;
    }
    const auto& v = isReverse_[ptgIndex];
    return static_cast<size_t>(pathIndex) < v.size() && v[pathIndex];
}

double CostEvaluatorReverseMotion::operator()(const MoveEdgeSE2_TPS& edge) const
{
    if (!isReverse(edge.ptgIndex, edge.ptgPathIndex)) { return 0; }
    return params_.reverseTimeCostFactor * edge.estimatedExecTime;
}
