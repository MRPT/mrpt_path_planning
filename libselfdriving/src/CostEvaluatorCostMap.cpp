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

    const float D = p.preferredClearanceDistance;

    // Find out required area and fill in with zeros:
    auto bbox = obsPts.boundingBox();

    bbox.min -= {D, D, 0.f};
    bbox.max += {D, D, 0.f};

    double defaultCost = .0;
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
            if (d < D)
            {
                const auto cost =
                    p.maxCost * std::pow(-0.99999 + 1. / (d / D), 0.1);
                ASSERT_GE_(cost, .0);

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
    double cost = .0;
    size_t n    = 0;

    auto lambdaAddPose = [this, &cost, &n](const mrpt::math::TPose2D& p) {
        const auto c = eval_single_pose(p);
        ASSERT_GE_(c, .0);
        cost += c;
        ++n;
    };

    // interpolated vs goal-end segments:
    if (edge.interpolatedPath.has_value())
    {
        for (const auto& p : *edge.interpolatedPath)
            lambdaAddPose(edge.stateFrom.pose + p);
    }
    else
    {
        lambdaAddPose(edge.stateFrom.pose);
        lambdaAddPose(edge.stateTo.pose);
    }

    return cost / n;
}

double CostEvaluatorCostMap::eval_single_pose(
    const mrpt::math::TPose2D& p) const
{
    const double* cell = costmap_.cellByPos(p.x, p.y);
    if (cell)
        return *cell;
    else
        return .0;
}
