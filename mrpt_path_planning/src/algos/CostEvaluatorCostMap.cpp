/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/opengl/CTexturedPlane.h>

using namespace mpp;

IMPLEMENTS_MRPT_OBJECT(CostEvaluatorCostMap, CostEvaluator, mpp)

CostEvaluatorCostMap::Parameters::Parameters() = default;

CostEvaluatorCostMap::Parameters::~Parameters() = default;

CostEvaluatorCostMap::Parameters CostEvaluatorCostMap::Parameters::FromYAML(
    const mrpt::containers::yaml& c)
{
    CostEvaluatorCostMap::Parameters p;
    p.load_from_yaml(c);
    return p;
}

mrpt::containers::yaml CostEvaluatorCostMap::Parameters::as_yaml()
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    MCP_SAVE(c, resolution);
    MCP_SAVE(c, preferredClearanceDistance);
    MCP_SAVE(c, maxCost);
    MCP_SAVE(c, useAverageOfPath);
    MCP_SAVE(c, maxRadiusFromRobot);

    return c;
}
void CostEvaluatorCostMap::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_REQ(c, resolution);
    MCP_LOAD_REQ(c, preferredClearanceDistance);
    MCP_LOAD_REQ(c, maxCost);
    MCP_LOAD_REQ(c, useAverageOfPath);
    MCP_LOAD_REQ(c, maxRadiusFromRobot);
}

CostEvaluatorCostMap::~CostEvaluatorCostMap() = default;

CostEvaluatorCostMap::Ptr CostEvaluatorCostMap::FromStaticPointObstacles(
    const mrpt::maps::CPointsMap&             obsPts,
    const CostEvaluatorCostMap::Parameters&   p,
    const std::optional<mrpt::math::TPose2D>& curRobotPose)
{
    auto cm     = CostEvaluatorCostMap::Create();
    cm->params_ = p;

    ASSERT_(!obsPts.empty());

    const float D = p.preferredClearanceDistance;

    // Find out required area and fill in with zeros:
    auto bbox = obsPts.boundingBox();

    bbox.min -= {D, D, 0.f};
    bbox.max += {D, D, 0.f};

    // optional limit to costmap area:
    if (p.maxRadiusFromRobot > 0)
    {
        ASSERT_(curRobotPose.has_value());
        const auto t = curRobotPose->translation();
        const auto R = p.maxRadiusFromRobot + D;

        bbox.min = mrpt::math::TPoint3Df(t.x - R, t.y - R, 0);
        bbox.max = mrpt::math::TPoint3Df(t.x + R, t.y + R, 0);
    }

    double defaultCost = .0;
    cm->costmap_.setSize(
        bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y, p.resolution,
        &defaultCost);

    // simple approach: for each cell, eval cost according to closest obstacle,
    // searching in a kd-tree:
    auto& g = cm->costmap_;
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

#if 0
    {
        mrpt::math::CMatrixDouble CM;
        g.getAsMatrix(CM);
        CM.saveToTextFile("costmap.txt");
    }
#endif

    return cm;
}

double CostEvaluatorCostMap::operator()(const MoveEdgeSE2_TPS& edge) const
{
    double cost = .0;
    size_t n    = 0;

    auto lambdaAddPose = [this, &cost, &n](const mrpt::math::TPose2D& p) {
        const auto c = eval_single_pose(p);
        ASSERT_GE_(c, .0);

        if (params_.useAverageOfPath)
        {
            cost += c;
            ++n;
        }
        else
        {
            if (c >= cost)
            {
                cost = c;
                n    = 1;
            }
        }
    };

    // interpolated vs goal-end segments:
    ASSERT_(!edge.interpolatedPath.empty());
    for (const auto& kv : edge.interpolatedPath)
        lambdaAddPose(edge.stateFrom.pose + kv.second);

    ASSERT_(n);

    return cost / n;
}

double CostEvaluatorCostMap::eval_single_pose(
    const mrpt::math::TPose2D& p) const
{
    const double* cell = costmap_.cellByPos(p.x, p.y);
    return cell ? *cell : .0;
}

mrpt::opengl::CSetOfObjects::Ptr CostEvaluatorCostMap::get_visualization() const
{
    const uint8_t COST_TRANSPARENCY_ALPHA = 0x80;
    const double  MIN_COST_TO_TRANSPARENT = 0.02;

    auto glObjs = mrpt::opengl::CSetOfObjects::Create();
    glObjs->setName("CostEvaluatorCostMap");
    auto glPlane = mrpt::opengl::CTexturedPlane::Create();
    glObjs->insert(glPlane);

    glPlane->setPlaneCorners(
        costmap_.getXMin(), costmap_.getXMax(), costmap_.getYMin(),
        costmap_.getYMax());

    const auto nCols = costmap_.getSizeX(), nRows = costmap_.getSizeY();

    mrpt::img::CImage gridRGB(nCols, nRows, mrpt::img::CH_RGB);
    mrpt::img::CImage gridALPHA(nCols, nRows, mrpt::img::CH_GRAY);

    gridRGB.filledRectangle(
        0, 0, nCols - 1, nRows - 1, mrpt::img::TColor::black());
    gridALPHA.filledRectangle(
        0, 0, nCols - 1, nRows - 1, mrpt::img::TColor::black());

    for (size_t icy = 0; icy < nRows; icy++)
    {
        for (size_t icx = 0; icx < nCols; icx++)
        {
            const double* c = costmap_.cellByIndex(icx, icy);
            if (!c) continue;
            const double val = *c;
            if (val < MIN_COST_TO_TRANSPARENT)
            {
                *gridALPHA(icx, icy) = 0x00;  // 100% transparent
            }
            else
            {
                *gridALPHA(icx, icy) = COST_TRANSPARENCY_ALPHA;

                const mrpt::img::TColor cellColor = mrpt::img::colormap(
                    mrpt::img::cmJET, val / params_.maxCost);

                uint8_t* bgr = gridRGB(icx, icy);
                bgr[0]       = cellColor.B;
                bgr[1]       = cellColor.G;
                bgr[2]       = cellColor.R;
            }
        }
    }

    glPlane->assignImage(gridRGB, gridALPHA);

    return glObjs;
}
