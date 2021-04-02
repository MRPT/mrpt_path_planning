/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>  // exception_to_str()
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/os.h>  // plugins
#include <selfdriving/TPS_RRTstar.h>
#include <selfdriving/viz.h>

#include <iostream>

static TCLAP::CmdLine cmd("plan-path");

static TCLAP::ValueArg<std::string> arg_obs_file(
    "o", "obstacles", "Input .txt file with obstacle points.", true, "",
    "obs.txt", cmd);

static TCLAP::ValueArg<std::string> arg_ptgs_file(
    "p", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

static TCLAP::ValueArg<std::string> arg_config_file_section(
    "", "config-section",
    "If loading from an INI file, the name of the section to load", false,
    "SelfDriving", "SelfDriving", cmd);

static TCLAP::ValueArg<std::string> arg_start_pose(
    "s", "start-pose", "Start 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

static TCLAP::ValueArg<std::string> arg_goal_pose(
    "g", "goal-pose", "Goal 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

static TCLAP::ValueArg<double> arg_min_step_len(
    "", "min-step-length", "Minimum step length [meters]", false, 0.25, "0.25",
    cmd);

static TCLAP::ValueArg<size_t> argMaxIterations(
    "", "max-iterations", "Maximum RRT iterations", false, 1000, "1000", cmd);

static TCLAP::ValueArg<unsigned int> argRandomSeed(
    "", "random-seed", "Pseudorandom generator seed (default: from time)",
    false, 0, "0", cmd);

static TCLAP::ValueArg<std::string> arg_plugins(
    "", "plugins",
    "Optional plug-in libraries to load, for externally-defined PTGs", false,
    "", "mylib.so", cmd);

static TCLAP::ValueArg<double> argSaveDebugVisualizationDecimation(
    "", "save-debug-visualization", "RRT* step frequency to save 3Dscene files",
    false, 0, "100", cmd);

static void do_plan_path()
{
    // Load obstacles:
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();
    if (!obsPts->load2D_from_text_file(arg_obs_file.getValue()))
        THROW_EXCEPTION_FMT(
            "Cannot read obstacle point cloud from: `%s`",
            arg_obs_file.getValue().c_str());

    auto obs = selfdriving::ObstacleSource::FromStaticPointcloud(obsPts);

    // Prepare planner input data:
    selfdriving::PlannerInput pi;

    pi.stateStart.pose.fromString(arg_start_pose.getValue());
    pi.stateGoal.pose.fromString(arg_goal_pose.getValue());
    pi.obstacles = obs;

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
    std::cout << "Obstacles : " << pi.obstacles->obstacles()->size()
              << " points\n";
    std::cout << "World bbox: " << pi.worldBboxMin.asString() << " - "
              << pi.worldBboxMax.asString() << "\n";

    // Do the path planning :
    selfdriving::TPS_RRTstar planner;

    // Enable time profiler:
    planner.profiler_.enable(true);

    // verbosity level:
    planner.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

    // Set planner required params:
    planner.params_.saveDebugVisualizationDecimation =
        argSaveDebugVisualizationDecimation.getValue();

    if (arg_min_step_len.isSet())
        planner.params_.minStepLength = arg_min_step_len.getValue();

    if (argMaxIterations.isSet())
        planner.params_.maxIterations = argMaxIterations.getValue();

    // PTGs config file:
    mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
    pi.ptgs.initFromConfigFile(cfg, arg_config_file_section.getValue());

    const selfdriving::PlannerOutput plan = planner.plan(pi);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
              << " overall edges, " << plan.motionTree.nodes().size()
              << " nodes\n";

    // Visualize:
    selfdriving::VisualizationOptions vizOpts;

    vizOpts.renderOptions.highlight_path_to_node_id = plan.goalNodeId;

    selfdriving::viz_nav_plan(plan, vizOpts);
}

int main(int argc, char** argv)
{
    try
    {
        if (!cmd.parse(argc, argv)) return 1;

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
