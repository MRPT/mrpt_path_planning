/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>  // exception_to_str()
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // plugins
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/TPS_RRTstar.h>
#include <selfdriving/algos/viz.h>

#include <fstream>
#include <iostream>

static TCLAP::CmdLine cmd("path-planner-cli");

static TCLAP::ValueArg<std::string> arg_obs_file(
    "o", "obstacles",
    "Input obstacles: either (1) a .txt file with obstacle points (one 'x y' "
    "pair per line), or (2) a .gridmap file, or a (3) png file with an "
    "gray-scale occupancy grid (*.png, *.bmp)",
    true, "", "obs.txt", cmd);

static TCLAP::ValueArg<std::string> argPlanner(
    "p", "planner", "Planner C++ class name to use", false,
    "selfdriving::TPS_Astar", "selfdriving::TPS_Astar", cmd);

static TCLAP::ValueArg<std::string> argVerbosity(
    "v", "verbose", "Verbosity level for path planner", false, "INFO",
    "ERROR|WARN|INFO|DEBUG", cmd);

static TCLAP::ValueArg<float> argObstaclesGridResolution(
    "", "obstacles-gridimage-resolution",
    "Only if --obstacles points to an image file, this sets the length of each "
    "pixel in meters.",
    false, 0.05, "0.05", cmd);

static TCLAP::ValueArg<std::string> arg_ptgs_file(
    "p", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

static TCLAP::ValueArg<std::string> argPlanner_yaml_file(
    "", "planner-parameters", "Input .yaml file with planner parameters", false,
    "", "tps-rrtstar.yaml", cmd);

static TCLAP::ValueArg<std::string> argPlanner_yaml_output_file(
    "", "write-planner-parameters",
    "If defined, it will save default planner params to a .yaml file and exit.",
    false, "", "tps-rrtstar.yaml", cmd);

static TCLAP::ValueArg<std::string> arg_config_file_section(
    "", "config-section",
    "If loading from an INI file, the name of the section to load", false,
    "SelfDriving", "SelfDriving", cmd);

static TCLAP::ValueArg<std::string> arg_start_pose(
    "s", "start-pose", "Start 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

static TCLAP::ValueArg<std::string> arg_start_vel(
    "", "start-vel", "Start 2D velocity", false, "[0 0 0]",
    "\"[vx vy omega_deg]\"", cmd);

static TCLAP::ValueArg<std::string> arg_goal_pose(
    "g", "goal-pose", "Goal 2D pose", true, "[0 0 0]", "\"[x y phi_deg]\"",
    cmd);

static TCLAP::ValueArg<std::string> arg_goal_vel(
    "", "goal-vel", "Goal 2D velocity", false, "", "\"[vx vy omega_deg]\"",
    cmd);

static TCLAP::ValueArg<unsigned int> argRandomSeed(
    "", "random-seed", "Pseudorandom generator seed (default: from time)",
    false, 0, "0", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "", "plugins",
    "Optional plug-in libraries to load, for externally-defined PTGs", false,
    "", "mylib.so", cmd);

static TCLAP::SwitchArg arg_costMap(
    "", "costmap", "Enable the default costmap from obstacle point clouds",
    cmd);

static mrpt::maps::CPointsMap::Ptr load_obstacles()
{
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();

    const auto sFile = arg_obs_file.getValue();
    ASSERT_FILE_EXISTS_(sFile);

    const auto sExt =
        mrpt::system::extractFileExtension(sFile, true /*ignore .gz*/);

    if (mrpt::system::strCmpI(sExt, "txt") ||
        mrpt::system::strCmpI(sExt, "pts"))
    {
        if (!obsPts->load2D_from_text_file(sFile))
            THROW_EXCEPTION_FMT(
                "Cannot read obstacle point cloud from: `%s`",
                arg_obs_file.getValue().c_str());
    }
    else if (mrpt::system::strCmpI(sExt, "gridmap"))
    {
        mrpt::io::CFileGZInputStream f(sFile);
        auto                         a = mrpt::serialization::archiveFrom(f);

        mrpt::maps::COccupancyGridMap2D grid;
        a >> grid;
        grid.getAsPointCloud(*obsPts);
    }
    else if (
        mrpt::system::strCmpI(sExt, "png") ||
        mrpt::system::strCmpI(sExt, "bmp"))
    {
        mrpt::maps::COccupancyGridMap2D grid;
        grid.loadFromBitmapFile(sFile, argObstaclesGridResolution.getValue());
        grid.getAsPointCloud(*obsPts);
    }

    return obsPts;
}

static void do_plan_path()
{
    // Load obstacles:
    mrpt::maps::CPointsMap::Ptr obsPts = load_obstacles();
    auto obs = selfdriving::ObstacleSource::FromStaticPointcloud(obsPts);

    // Prepare planner input data:
    selfdriving::PlannerInput pi;

    pi.stateStart.pose.fromString(arg_start_pose.getValue());
    if (arg_start_vel.isSet())
        pi.stateStart.vel.fromString(arg_start_vel.getValue());

    pi.stateGoal.pose.fromString(arg_goal_pose.getValue());
    if (arg_goal_vel.isSet())
        pi.stateGoal.vel.fromString(arg_goal_vel.getValue());

    pi.obstacles.emplace_back(obs);

    auto bbox = obs->obstacles()->boundingBox();

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin = mrpt::math::TPoint3Df(1.0, 1.0, .0);
        const auto ptStart    = mrpt::math::TPoint3Df(
            pi.stateStart.pose.x, pi.stateStart.pose.y, 0);
        const auto ptGoal =
            mrpt::math::TPoint3Df(pi.stateGoal.pose.x, pi.stateGoal.pose.y, 0);
        bbox.updateWithPoint(ptStart - bboxMargin);
        bbox.updateWithPoint(ptStart + bboxMargin);
        bbox.updateWithPoint(ptGoal - bboxMargin);
        bbox.updateWithPoint(ptGoal + bboxMargin);
    }

    pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
    pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

    std::cout << "Start pose: " << pi.stateStart.pose.asString() << "\n";
    std::cout << "Goal pose : " << pi.stateGoal.pose.asString() << "\n";
    std::cout << "Obstacles : " << obs->obstacles()->size() << " points\n";
    std::cout << "World bbox: " << pi.worldBboxMin.asString() << " - "
              << pi.worldBboxMax.asString() << "\n";

    // Do the path planning :
    selfdriving::Planner::Ptr planner =
        std::dynamic_pointer_cast<selfdriving::Planner>(
            mrpt::rtti::classFactory(argPlanner.getValue()));

    if (!planner)
    {
        THROW_EXCEPTION_FMT(
            "Given classname '%s' does not seem to be a known C++ class "
            "implementing `Planner",
            argPlanner.getValue().c_str());
    }

    // Enable time profiler:
    planner->profiler_.enable(true);

    if (arg_costMap.isSet())
    {
        // cost map:
        auto costmap =
            selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                *obsPts);

        planner->costEvaluators_.push_back(costmap);
    }

    // verbosity level:
    planner->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosity.getValue()));

    // Set planner required params:
    if (argPlanner_yaml_file.isSet())
    {
        const auto sFile = argPlanner_yaml_file.getValue();
        ASSERT_FILE_EXISTS_(sFile);
        const auto c = mrpt::containers::yaml::FromFile(sFile);
        planner->params_from_yaml(c);
        std::cout << "Loaded these planner params:\n";
        planner->params_as_yaml().printAsYAML();
    }

    // PTGs config file:
    mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
    pi.ptgs.initFromConfigFile(cfg, arg_config_file_section.getValue());

    const selfdriving::PlannerOutput plan = planner->plan(pi);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
              << " overall edges, " << plan.motionTree.nodes().size()
              << " nodes\n";

    // Visualize:
    selfdriving::VisualizationOptions vizOpts;

    vizOpts.renderOptions.highlight_path_to_node_id = plan.goalNodeId;
    vizOpts.renderOptions.color_normal_edge         = {0xb0b0b0, 0x20};  // RGBA
    // vizOpts.renderOptions.width_normal_edge         = 0;  // hidden
    // vizOpts.renderOptions.showEdgeWeights           = true;

    selfdriving::viz_nav_plan(plan, vizOpts);
}

int main(int argc, char** argv)
{
    try
    {
        bool cmdsOk = cmd.parse(argc, argv);

        if (argPlanner_yaml_output_file.isSet())
        {
            selfdriving::TPS_RRTstar_Parameters defaults;
            const auto                          c = defaults.as_yaml();
            const auto    sFile = argPlanner_yaml_output_file.getValue();
            std::ofstream fileYaml(sFile);
            ASSERT_(fileYaml.is_open());
            c.printAsYAML(fileYaml);
            std::cout << "Wrote file: " << sFile << std::endl;
            return 0;
        }

        if (!cmdsOk) return 1;

        if (arg_plugins.isSet())
        {
            std::string loadErrors;
            if (!mrpt::system::loadPluginModules(
                    arg_plugins.getValue(), loadErrors))
            {
                std::cerr << "Could not load plugins, error: " << loadErrors;
                return 1;
            }
        }

        if (argRandomSeed.isSet())
        {
            mrpt::random::getRandomGenerator().randomize(
                argRandomSeed.getValue());
        }

        do_plan_path();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e);
        return 1;
    }
}
