/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/TrajectoriesAndRobotShape.h>

using namespace mpp;

void TrajectoriesAndRobotShape::clear() { *this = TrajectoriesAndRobotShape(); }

void TrajectoriesAndRobotShape::initFromConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s)
{
    MRPT_START

    const auto   ptg_cache_files_directory = std::string(".");
    unsigned int PTG_COUNT = c.read_int(s, "PTG_COUNT", 0, true);

    // Load robot shape: 1/2 polygon
    // ---------------------------------------------
    mrpt::math::CPolygon robShape;

    std::vector<float> xs, ys;
    c.read_vector(s, "RobotModel_shape2D_xs", std::vector<float>(), xs, false);
    c.read_vector(s, "RobotModel_shape2D_ys", std::vector<float>(), ys, false);
    ASSERTMSG_(
        xs.size() == ys.size(),
        "Config parameters `RobotModel_shape2D_xs` and `RobotModel_shape2D_ys` "
        "must have the same length!");
    if (!xs.empty())
    {
        auto& poly = robotShape.emplace<mrpt::math::TPolygon2D>();
        for (size_t i = 0; i < xs.size(); i++)
        {
            poly.emplace_back(xs[i], ys[i]);
            robShape.AddVertex(xs[i], ys[i]);
        }
    }

    // Load robot shape: 2/2 circle
    // ---------------------------------------------
    if (const double robot_radius =
            c.read_double(s, "RobotModel_circular_shape_radius", -1.0, false);
        robot_radius > 0)
    {
        auto& r = robotShape.emplace<robot_radius_t>();
        r       = robot_radius;
    }

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
            ptg_polygon->setRobotShape(robShape);
        }

        if (auto* ptg_circ = dynamic_cast<mrpt::nav::CPTG_RobotShape_Circular*>(
                new_ptg.get());
            ptg_circ)
        {
            // Set it:
            ptg_circ->setRobotShapeRadius(std::get<robot_radius_t>(robotShape));
        }

        // Init:
        new_ptg->initialize(
            mrpt::format(
                "%s/ptg_grid_%03u.dat.gz", ptg_cache_files_directory.c_str(),
                n),
            false /*verbose*/
        );
    }
    initialized_ = true;
    MRPT_END
}

#if 0
void TrajectoriesAndRobotShape::initFromYAML(const mrpt::containers::yaml& node)
{
    MRPT_START
    THROW_EXCEPTION("Write me!");
    initialized_ = false;
    MRPT_END
}
#endif

bool mpp::obstaclePointCollides(
    const mrpt::math::TPoint2D&      obstacleWrtRobot,
    const TrajectoriesAndRobotShape& trs)
{
    return trs.ptgs.at(0)->isPointInsideRobotShape(
        obstacleWrtRobot.x, obstacleWrtRobot.y);
}
