/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/PlannerTypes.h>

using namespace selfdrive;

ObstacleSource::~ObstacleSource() = default;

ObstacleSource::ObstacleSource(
    const mrpt::maps::CSimplePointsMap& staticObstacles)
{
    static_obs_ =
        std::make_shared<mrpt::maps::CSimplePointsMap>(staticObstacles);
}

mrpt::maps::CSimplePointsMap::Ptr ObstacleSource::obstacles(
    [[maybe_unused]] mrpt::system::TTimeStamp t)
{
    MRPT_START

    if (static_obs_) return static_obs_;

    THROW_EXCEPTION(
        "Must either reimplement obstacles() or use ctor from static "
        "obstacles");

    MRPT_END
}

void TrajectoriesAndRobotShape::initFromConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s)
{
    MRPT_START

    const auto   ptg_cache_files_directory = std::string(".");
    unsigned int PTG_COUNT = c.read_int(s, "PTG_COUNT", 0, true);

    // Load robot shape: 1/2 polygon
    // ---------------------------------------------
    std::vector<float> xs, ys;
    c.read_vector(s, "RobotModel_shape2D_xs", std::vector<float>(), xs, false);
    c.read_vector(s, "RobotModel_shape2D_ys", std::vector<float>(), ys, false);
    ASSERTMSG_(
        xs.size() == ys.size(),
        "Config parameters `RobotModel_shape2D_xs` and `RobotModel_shape2D_ys` "
        "must have the same length!");
    if (!xs.empty())
    {
        robotShape.clear();
        for (size_t i = 0; i < xs.size(); i++)
            robotShape.AddVertex(xs[i], ys[i]);
    }

    // Load robot shape: 2/2 circle
    // ---------------------------------------------
    robotShapeCircularRadius =
        c.read_double(s, "RobotModel_circular_shape_radius", .0, false);
    ASSERT_(robotShapeCircularRadius >= .0);

    // Load PTGs from file:
    // ---------------------------------------------
    // Free previous PTGs:
    ptgs.clear();
    ptgs.resize(PTG_COUNT);

    for (unsigned int n = 0; n < PTG_COUNT; n++)
    {
        // Factory:
        const std::string sPTGName =
            c.read_string(s, mrpt::format("PTG%u_Type", n), "", true);
        auto new_ptg = mrpt::nav::CParameterizedTrajectoryGenerator::CreatePTG(
            sPTGName, c, s, mrpt::format("PTG%u_", n));

        ptgs[n] = new_ptg;

        // Initialize PTGs:
        new_ptg->deinitialize();

        // Polygonal robot shape?
        if (auto* ptg_polygon =
                dynamic_cast<mrpt::nav::CPTG_RobotShape_Polygonal*>(
                    new_ptg.get());
            ptg_polygon)
        {
            // Set it:
            ptg_polygon->setRobotShape(robotShape);
        }

        if (auto* ptg_circ = dynamic_cast<mrpt::nav::CPTG_RobotShape_Circular*>(
                new_ptg.get());
            ptg_circ)
        {
            // Set it:
            ptg_circ->setRobotShapeRadius(robotShapeCircularRadius);
        }

        // Init:
        new_ptg->initialize(
            mrpt::format(
                "%s/ReacNavGrid_%03u.dat.gz", ptg_cache_files_directory.c_str(),
                n),
            false /*verbose*/
        );
    }

    MRPT_END
}

mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState
    NavPlanAction::getPTGDynState() const
{
    mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState newDyn;

    newDyn.relTarget   = state_to.pose - state_from.pose;
    newDyn.curVelLocal = state_from.vel;
    // Global to local velocity:
    newDyn.curVelLocal.rotate(-state_from.pose.phi);

    MRPT_TODO("Support stop at final pose?");
    newDyn.targetRelSpeed = 1.0;

    return newDyn;
}
