/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/opengl/CTexturedPlane.h>

#include <algorithm>
#include <cmath>

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

namespace
{
// Builds the set of robot-frame points at which the footprint cost is sampled.
// For a polygon: its vertices plus points subdividing each edge finer than the
// costmap resolution, so the max cost picks up the footprint side/corner
// closest to an obstacle. For a radius: a ring of points at that radius. The
// maximum costmap value over these points (transformed to a path pose) then
// reflects the true footprint clearance, not just the reference-point
// clearance.
std::vector<mrpt::math::TPoint2D> buildShapeSamples(
    const mpp::RobotShape& shape, double resolution)
{
    std::vector<mrpt::math::TPoint2D> pts;
    const double                      step = std::max(0.01, resolution);

    if (const auto* poly = std::get_if<mrpt::math::TPolygon2D>(&shape))
    {
        const auto&  v = *poly;
        const size_t n = v.size();
        if (n < 2) return pts;
        for (size_t i = 0; i < n; i++)
        {
            const auto&  a   = v[i];
            const auto&  b   = v[(i + 1) % n];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            const int    nSeg =
                std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int k = 0; k < nSeg;
                 k++)  // include a, exclude b (next edge's a)
            {
                const double t = static_cast<double>(k) / nSeg;
                pts.emplace_back(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
            }
        }
    }
    else if (const auto* radius = std::get_if<mpp::robot_radius_t>(&shape))
    {
        const double r = *radius;
        if (r <= 0) return pts;
        const int nSeg =
            std::max(8, static_cast<int>(std::ceil(2 * M_PI * r / step)));
        for (int k = 0; k < nSeg; k++)
        {
            const double a = 2 * M_PI * k / nSeg;
            pts.emplace_back(r * std::cos(a), r * std::sin(a));
        }
    }
    // std::monostate -> empty (legacy origin-only sampling)
    return pts;
}
}  // namespace

CostEvaluatorCostMap::Ptr CostEvaluatorCostMap::FromStaticPointObstacles(
    const mrpt::maps::CPointsMap&             obsPts,
    const CostEvaluatorCostMap::Parameters&   p,
    const std::optional<mrpt::math::TPose2D>& curRobotPose,
    const RobotShape&                         robotShape)
{
    auto cm     = CostEvaluatorCostMap::Create();
    cm->params_ = p;

    cm->shapeSamples_ = buildShapeSamples(robotShape, p.resolution);

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
                // Smooth quadratic decay: maxCost at d=0, zero at d=D.
                // No singularity and good gradient across the full
                // clearance zone.
                const double nd   = 1.0 - d / D;  // in [0,1]
                const auto   cost = p.maxCost * nd * nd;
                ASSERT_GE_(cost, .0);

                double* cell = g.cellByIndex(cx, cy);
                ASSERT_(cell);
                *cell = cost;
            }
        }
    }

    // Ensure cells containing obstacle points have exactly maxCost.
    // The loop above computes cost from cell centers, which may be slightly
    // offset from the obstacle, yielding cost < maxCost at the obstacle cell.
    {
        const auto& xs = obsPts.getPointsBufferRef_x();
        const auto& ys = obsPts.getPointsBufferRef_y();
        for (size_t i = 0; i < xs.size(); i++)
        {
            double* cell = g.cellByPos(xs[i], ys[i]);
            if (cell) *cell = p.maxCost;
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

    auto lambdaAddPose = [this, &cost, &n](const mrpt::math::TPose2D& p)
    {
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
    // Legacy behavior: sample only the trajectory reference point.
    if (shapeSamples_.empty())
    {
        const double* cell = costmap_.cellByPos(p.x, p.y);
        return cell ? *cell : .0;
    }

    // Footprint-aware: max cost over the robot shape at this pose. The costmap
    // cost decreases with distance to the nearest obstacle, so the maximum over
    // the footprint boundary is the cost at the footprint point closest to an
    // obstacle, i.e. the true clearance cost.
    const double c       = std::cos(p.phi);
    const double s       = std::sin(p.phi);
    double       maxCost = .0;
    for (const auto& v : shapeSamples_)
    {
        const double  wx   = p.x + v.x * c - v.y * s;
        const double  wy   = p.y + v.x * s + v.y * c;
        const double* cell = costmap_.cellByPos(wx, wy);
        if (cell && *cell > maxCost) maxCost = *cell;
    }
    return maxCost;
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

mrpt::maps::COccupancyGridMap2D::Ptr
    CostEvaluatorCostMap::get_visualization_as_grid() const
{
    auto grid = mrpt::maps::COccupancyGridMap2D::Create();

    grid->setSize(
        costmap_.getXMin(), costmap_.getXMax(), costmap_.getYMin(),
        costmap_.getYMax(), costmap_.getResolution());

    ASSERT_EQUAL_(grid->getSizeX(), costmap_.getSizeX());
    ASSERT_EQUAL_(grid->getSizeY(), costmap_.getSizeY());

    /* From: https://github.com/ros2/rviz/blob/rolling/docs/FEATURES.md
     *
     * Costmap: Paint valid points between 1 and 98 from blue to red. Paint
     * points with value 0 in black, points with value 99 in cyan (obstacle
     * value) and points with value 100 in purple (lethal obstacle). The valid
     * value -1 is painted in a blueish, greenish, grayish color. Invalid points
     * between 101 and 127 are painted in green, while invalid negative numbers
     * are painted in shades from red to yellow.
     *
     * So we will use:
     * -1=free space
     * 1-98: obstacles.
     */

    const double MIN_COST_TO_TRANSPARENT = 0.02;

    const auto nCols = costmap_.getSizeX(), nRows = costmap_.getSizeY();

    mrpt::img::CImage gridRGB(nCols, nRows, mrpt::img::CH_RGB);
    mrpt::img::CImage gridALPHA(nCols, nRows, mrpt::img::CH_GRAY);

    for (size_t icy = 0; icy < nRows; icy++)
    {
        auto* row = reinterpret_cast<int8_t*>(grid->getRow(icy));
        ASSERT_(row);

        for (size_t icx = 0; icx < nCols; icx++)
        {
            const double* c = costmap_.cellByIndex(icx, icy);
            if (!c) continue;  // should not happen?
            const double val = *c;
            if (val < MIN_COST_TO_TRANSPARENT)
            {
                row[icx] = -1;  // transparent, free space
            }
            else
            {
                const double f = val / params_.maxCost;
                row[icx]       = std::min<int8_t>(
                    98, std::max<int8_t>(1, static_cast<int8_t>(1 + 97 * f)));
            }
        }
    }

    return grid;
}
