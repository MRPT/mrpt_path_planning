/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/data/TrajectoriesAndRobotShape.h>
#include <mpp/ptgs/DiffDrive_C.h>
#include <mrpt/system/os.h>

#include <cstdlib>
#include <filesystem>
#include <functional>

using namespace mpp;

namespace
{
// Directory for PTG collision grid cache files ("." if it cannot be created).
std::string ptgCacheDirectory()
{
    std::string base;
    if (const char* xdg = std::getenv("XDG_CACHE_HOME"); xdg && *xdg)
    {
        base = xdg;
    }
    else if (const char* home = std::getenv("HOME"); home && *home)
    {
        base = std::string(home) + "/.cache";
    }
    if (base.empty()) { return "."; }
    const std::string dir = base + "/mrpt_path_planning";
    // Best effort: the cache is optional, so never throw from here.
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec || !std::filesystem::is_directory(dir, ec)) { return "."; }
    return dir;
}

// Hash of all config keys defining PTG #n and the robot shape, to name its
// cache file.
size_t ptgConfigHash(
    const mrpt::config::CConfigFileBase& c, const std::string& section,
    unsigned int n)
{
    const std::string prefix = mrpt::format("PTG%u_", n);
    std::string       all;
    for (const auto& key : c.keys(section))
    {
        if (key.rfind(prefix, 0) == 0 || key.rfind("RobotModel_", 0) == 0)
        {
            all += key + "=" + c.read_string(section, key, "") + ";";
        }
    }
    return std::hash<std::string>{}(all);
}
}  // namespace

void TrajectoriesAndRobotShape::clear() { *this = TrajectoriesAndRobotShape(); }

void TrajectoriesAndRobotShape::initFromConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s, bool initializePTGs)
{
    MRPT_START

    const auto ptg_cache_files_directory =
        initializePTGs ? ptgCacheDirectory() : std::string();
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
            robShape.add_vertex(xs[i], ys[i]);
        }
    }

    // Load robot shape: 2/2 circle
    // ---------------------------------------------
    if (const double robot_radius =
            c.read_double(s, "RobotModel_circular_shape_radius", -1.0, false);
        robot_radius > 0)
    {
        ASSERTMSG_(
            xs.empty(),
            "Both a polygonal (RobotModel_shape2D_*) and a circular "
            "(RobotModel_circular_shape_radius) robot shape are defined: "
            "define only one.");
        auto& r = robotShape.emplace<robot_radius_t>();
        r       = robot_radius;
    }

    minTurningRadius =
        c.read_double(s, "RobotModel_min_turning_radius", 0.0, false);
    ASSERT_GE_(minTurningRadius, 0.0);

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

        // Kinematic consistency with the vehicle (see minTurningRadius):
        if (const auto* c_ptg =
                dynamic_cast<const ptg::DiffDrive_C*>(new_ptg.get());
            c_ptg && minTurningRadius > 0 && c_ptg->getMax_W() > 0)
        {
            const double R = c_ptg->getMax_V() / c_ptg->getMax_W();
            ASSERTMSG_(
                R >= minTurningRadius * (1.0 - 1e-3),
                mrpt::format(
                    "PTG%u has a turning radius (v_max/w_max) of %.3f m, "
                    "smaller than RobotModel_min_turning_radius=%.3f m: plans "
                    "would not be drivable by the vehicle.",
                    n, R, minTurningRadius));
        }

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
        if (initializePTGs)
        {
            new_ptg->initialize(
                mrpt::format(
                    "%s/ptg_grid_%016zx.dat.gz",
                    ptg_cache_files_directory.c_str(), ptgConfigHash(c, s, n)),
                false /*verbose*/
            );
        }
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
