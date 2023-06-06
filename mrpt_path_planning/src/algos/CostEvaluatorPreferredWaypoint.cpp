/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mrpt/opengl/CDisk.h>

using namespace mpp;

IMPLEMENTS_MRPT_OBJECT(CostEvaluatorPreferredWaypoint, CostEvaluator, mpp)

CostEvaluatorPreferredWaypoint::Parameters::Parameters() = default;

CostEvaluatorPreferredWaypoint::Parameters::~Parameters() = default;

CostEvaluatorPreferredWaypoint::Parameters
    CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
        const mrpt::containers::yaml& c)
{
    CostEvaluatorPreferredWaypoint::Parameters p;
    p.load_from_yaml(c);
    return p;
}

mrpt::containers::yaml CostEvaluatorPreferredWaypoint::Parameters::as_yaml()
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    MCP_SAVE(c, waypointInfluenceRadius);
    MCP_SAVE(c, costScale);
    MCP_SAVE(c, useAverageOfPath);

    return c;
}
void CostEvaluatorPreferredWaypoint::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    ASSERT_(c.isMap());

    MCP_LOAD_REQ(c, waypointInfluenceRadius);
    MCP_LOAD_REQ(c, costScale);
    MCP_LOAD_REQ(c, useAverageOfPath);
}

CostEvaluatorPreferredWaypoint::~CostEvaluatorPreferredWaypoint() = default;

void CostEvaluatorPreferredWaypoint::setPreferredWaypoints(
    const std::vector<mrpt::math::TPoint2D>& pts)
{
    waypoints_.clear();
    for (const auto& pt : pts) waypoints_.insertPoint(pt.x, pt.y);

    // build 2D KD-tree now:
    waypoints_.kdTreeEnsureIndexBuilt2D();
}

double CostEvaluatorPreferredWaypoint::operator()(
    const MoveEdgeSE2_TPS& edge) const
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

double CostEvaluatorPreferredWaypoint::eval_single_pose(
    const mrpt::math::TPose2D& p) const
{
// indices-squaredDistances list:
#if NANOFLANN_VERSION >= 0x150
    std::vector<nanoflann::ResultItem<size_t, float>> nearWps;
#else
    std::vector<std::pair<size_t, float>> nearWps;
#endif

    const auto inflRadius = params_.waypointInfluenceRadius;

    waypoints_.kdTreeRadiusSearch2D(
        p.x, p.y, mrpt::square(inflRadius), nearWps);

    double cost = params_.costScale;
    for (const auto& wpIdxSqrDist : nearWps)
    {
        const float dist = sqrt(wpIdxSqrDist.second);
        if (dist > inflRadius || dist < 0) continue;

        // Decrease cost: prefer to pass thru these areas.
        cost -= params_.costScale * (1.0f - sqrt(dist / inflRadius));
    }

    return std::max(.0, cost);
}

mrpt::opengl::CSetOfObjects::Ptr
    CostEvaluatorPreferredWaypoint::get_visualization() const
{
    auto glObjs = mrpt::opengl::CSetOfObjects::Create();
    glObjs->setName("CostEvaluatorPreferredWaypoint");

    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        float x, y;
        waypoints_.getPoint(i, x, y);

        auto glDisk = mrpt::opengl::CDisk::Create();
        glDisk->setColor_u8(0x20, 0x20, 0xff, 0x20);
        glDisk->setDiskRadius(
            params_.waypointInfluenceRadius,
            params_.waypointInfluenceRadius * 0.90);
        glDisk->setLocation(x, y, 0);
        glObjs->insert(glDisk);
    }
    return glObjs;
}
