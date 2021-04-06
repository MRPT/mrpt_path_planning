/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <selfdriving/CostEvaluatorCostMap.h>

using namespace selfdriving;

CostEvaluatorCostMap::~CostEvaluatorCostMap() = default;

CostEvaluatorCostMap CostEvaluatorCostMap::FromStaticPointObstacles(
    const mrpt::maps::CPointsMap& obsPts, const CostMapParameters& p)
{
    CostEvaluatorCostMap cm;
    ASSERT_(!obsPts.empty());

    // Find out required area and fill in with zeros:
    const auto bbox        = obsPts.boundingBox();
    double     defaultCost = .0;
    cm.costmap_.setSize(
        bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y, p.resolution,
        &defaultCost);

    // simple approach: for each cell, eval cost according to closest obstacle,
    // searching in a kd-tree:
    auto& g = cm.costmap_;
    for (unsigned int cy = 0; cy < g.getSizeY(); cy++)
    {
        const float y = g.idx2y(cy);
        for (unsigned int cx = 0; cx < g.getSizeX(); cx++)
        {
            const float x = g.idx2x(cx);
            const auto d = std::sqrt(obsPts.kdTreeClosestPoint2DsqrError(x, y));
            if (d < p.preferredClearanceDistance)
            {
                const auto cost =
                    p.maxCost *
                    (1.0 - mrpt::square(d / p.preferredClearanceDistance));

                double* cell = g.cellByIndex(cx, cy);
                ASSERT_(cell);
                *cell = cost;
            }
        }
    }

    if (0)
    {
        mrpt::math::CMatrixDouble CM;
        g.getAsMatrix(CM);
        CM.saveToTextFile("costmap.txt");
    }

    return cm;
}

double CostEvaluatorCostMap::operator()(const MoveEdgeSE2_TPS& edge) const
{
    double maxCost = .0;

    auto lambdaAddPose = [&](const mrpt::math::TPose2D& p) {
        mrpt::keep_max(maxCost, eval_single_pose(p));
    };

    // start:
    lambdaAddPose(edge.stateFrom.pose);

    // intermediary points:
    if (edge.interpolatedPath.has_value())
        for (const auto& p : *edge.interpolatedPath) lambdaAddPose(p);

    // end
    lambdaAddPose(edge.stateTo.pose);

    return maxCost;
}

double CostEvaluatorCostMap::eval_single_pose(
    const mrpt::math::TPose2D& p) const
{
    const double* cell = costmap_.cellByPos(p.x, p.y);
    return cell ? *cell : .0;
}
