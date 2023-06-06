/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/algos/viz.h>
#include <mpp/data/Waypoints.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>  // exception_to_str()
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>  // intervalFormat()
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>  // plugins
#include <mrpt/version.h>

#include <fstream>
#include <iostream>

TCLAP::CmdLine cmd("path-planner-cli");

TCLAP::ValueArg<std::string> arg_obs_file(
    "o", "obstacles",
    "Input obstacles: either (1) a .txt file with obstacle points (one 'x y' "
    "pair per line), or (2) a .gridmap file, or (3) a ROS MAP YAML file, or a "
    "(4) png file with an gray-scale occupancy grid (*.png, *.bmp)",
    true, "", "obs.txt", cmd);

TCLAP::ValueArg<std::string> argPlanner(
    "p", "planner", "Planner C++ class name to use", false, "mpp::TPS_Astar",
    "mpp::TPS_Astar", cmd);

TCLAP::ValueArg<std::string> argVerbosity(
    "v", "verbose", "Verbosity level for path planner", false, "INFO",
    "ERROR|WARN|INFO|DEBUG", cmd);

TCLAP::ValueArg<float> argObstaclesGridResolution(
    "", "obstacles-gridimage-resolution",
    "Only if --obstacles points to an image file, this sets the length of each "
    "pixel in meters.",
    false, 0.05, "0.05", cmd);

TCLAP::ValueArg<std::string> arg_ptgs_file(
    "c", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

TCLAP::ValueArg<std::string> argPlanner_yaml_file(
    "", "planner-parameters", "Input .yaml file with planner parameters", false,
    "", "tps-astar.yaml", cmd);

TCLAP::ValueArg<std::string> argPlanner_yaml_output_file(
    "", "write-planner-parameters",
    "If defined, it will save default planner params to a .yaml file and exit.",
    false, "", "tps-astar.yaml", cmd);

TCLAP::ValueArg<std::string> arg_config_file_section(
    "", "config-section",
    "If loading from an INI file, the name of the section to load", false,
    "SelfDriving", "SelfDriving", cmd);

TCLAP::ValueArg<std::string> arg_start_pose(
    "s", "start-pose", "Start 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

TCLAP::ValueArg<std::string> arg_start_vel(
    "", "start-vel", "Start 2D velocity", false, "[0 0 0]",
    "\"[vx vy omega_deg]\"", cmd);

TCLAP::ValueArg<std::string> arg_goal_pose(
    "g", "goal-pose", "Goal 2D pose or point", true, "[0 0 0]",
    "\"[x y phi_deg]\" or \"[x y]\"", cmd);

TCLAP::ValueArg<double> argBBoxMargin(
    "", "bbox-margin", "Margin to add to the start-goal bbox poses", false, 1.0,
    "A distance [meters]", cmd);

TCLAP::ValueArg<std::string> arg_goal_vel(
    "", "goal-vel", "Goal 2D velocity", false, "", "\"[vx vy omega_deg]\"",
    cmd);

TCLAP::ValueArg<unsigned int> argRandomSeed(
    "", "random-seed", "Pseudorandom generator seed (default: from time)",
    false, 0, "0", cmd);

TCLAP::ValueArg<std::string> arg_plugins(
    "", "plugins",
    "Optional plug-in libraries to load, for externally-defined PTGs", false,
    "", "mylib.so", cmd);

TCLAP::ValueArg<std::string> arg_costMap(
    "", "costmap-obstacles",
    "Creates a costmap from obstacle point clouds with the given parameters "
    "from a YAML file.",
    false, "costmap.yaml", "costmap.yaml", cmd);

TCLAP::ValueArg<std::string> arg_waypoints(
    "", "waypoints",
    "This creates a preferred-waypoints costlayer, from the waypoint list in a "
    "given YAML file.",
    false, "waypoints.yaml", "waypoints.yaml", cmd);

TCLAP::ValueArg<std::string> arg_waypointsParams(
    "", "waypoints-parameters",
    "If --waypoints is also set, this loads the preferred waypoints costlayer "
    "parameters from a YAML file.",
    false, "waypoints-parameters.yaml", "waypoints-parameters.yaml", cmd);

TCLAP::SwitchArg arg_showTree(
    "", "show-tree",
    "Shows the whole search tree instead of just the best path", cmd);

TCLAP::SwitchArg arg_noRefine(
    "", "no-refine", "Skips the post-plan refine stage", cmd);

TCLAP::SwitchArg arg_showEdgeWeights(
    "", "show-edge-weights", "Shows the weight of path edges", cmd);

TCLAP::SwitchArg arg_printPathEdges(
    "", "print-path-edges", "Prints details on the found planned path edges",
    cmd);

TCLAP::ValueArg<std::string> arg_InterpolatePath(
    "", "save-interpolated-path",
    "Interpolates the path and saves it into a .csv file", false, "path.csv",
    "path.csv", cmd);

TCLAP::SwitchArg arg_playAnimation(
    "", "play-animation",
    "Shows the GUI with an animation of the vehicle moving along the path",
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
    else if (mrpt::system::strCmpI(sExt, "yaml"))
    {
#if MRPT_VERSION >= 0x250
        mrpt::maps::COccupancyGridMap2D grid;
        bool readOk = grid.loadFromROSMapServerYAML(sFile);
        ASSERT_(readOk);
        grid.getAsPointCloud(*obsPts);
#else
        THROW_EXCEPTION("Loading ROS YAML map files requires MRPT >=2.5.0");
#endif
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
    auto obs = mpp::ObstacleSource::FromStaticPointcloud(obsPts);

    // Prepare planner input data:
    mpp::PlannerInput pi;

    pi.stateStart.pose.fromString(arg_start_pose.getValue());
    if (arg_start_vel.isSet())
        pi.stateStart.vel.fromString(arg_start_vel.getValue());

    pi.stateGoal.state = mpp::PoseOrPoint::FromString(arg_goal_pose.getValue());
    if (arg_goal_vel.isSet())
        pi.stateGoal.vel.fromString(arg_goal_vel.getValue());

    pi.obstacles.emplace_back(obs);

    auto bbox = obs->obstacles()->boundingBox();

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin = mrpt::math::TPoint3Df(
            argBBoxMargin.getValue(), argBBoxMargin.getValue(), .0);
        const auto ptStart = mrpt::math::TPoint3Df(
            pi.stateStart.pose.x, pi.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(
            pi.stateGoal.asSE2KinState().pose.x,
            pi.stateGoal.asSE2KinState().pose.y, 0);
        bbox.updateWithPoint(ptStart - bboxMargin);
        bbox.updateWithPoint(ptStart + bboxMargin);
        bbox.updateWithPoint(ptGoal - bboxMargin);
        bbox.updateWithPoint(ptGoal + bboxMargin);
    }

    pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
    pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

    std::cout << "Start state: " << pi.stateStart.asString() << "\n";
    std::cout << "Goal state : " << pi.stateGoal.asString() << "\n";
    std::cout << "Obstacles  : " << obs->obstacles()->size() << " points\n";
    std::cout << "World bbox : " << pi.worldBboxMin.asString() << " - "
              << pi.worldBboxMax.asString() << "\n";

    // Do the path planning :
    mpp::Planner::Ptr planner = std::dynamic_pointer_cast<mpp::Planner>(
        mrpt::rtti::classFactory(argPlanner.getValue()));

    if (!planner)
    {
        THROW_EXCEPTION_FMT(
            "Given classname '%s' does not seem to be a known C++ class "
            "implementing `Planner",
            argPlanner.getValue().c_str());
    }

    // Enable time profiler:
    planner->profiler_().enable(true);

    if (arg_costMap.isSet())
    {
        // cost map:
        const auto costMapParams =
            mpp::CostEvaluatorCostMap::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(arg_costMap.getValue()));

        auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
            *obsPts, costMapParams, pi.stateStart.pose);

        planner->costEvaluators_.push_back(costmap);
    }

    // Preferred waypoints:
    auto wpParams = mpp::CostEvaluatorPreferredWaypoint::Parameters();
    if (arg_waypointsParams.isSet())
    {
        wpParams = mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(arg_waypointsParams.getValue()));
    }

    if (arg_waypoints.isSet())
    {
        const auto wps = mpp::WaypointSequence::FromYAML(
            mrpt::containers::yaml::FromFile(arg_waypoints.getValue()));

        std::vector<mrpt::math::TPoint2D> lstPts;
        for (const auto& wp : wps.waypoints) lstPts.emplace_back(wp.target);

        auto costEval     = mpp::CostEvaluatorPreferredWaypoint::Create();
        costEval->params_ = wpParams;
        costEval->setPreferredWaypoints(lstPts);
        planner->costEvaluators_.push_back(costEval);
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

    // Insert custom progress callback:
    planner->progressCallback_ = [](const mpp::ProgressCallbackData& pcd) {
        std::cout << "[progressCallback] bestCostFromStart: "
                  << pcd.bestCostFromStart
                  << " bestCostToGoal: " << pcd.bestCostToGoal
                  << " bestPathLength: " << pcd.bestPath.size() << std::endl;
    };

    // PTGs config file:
    mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
    pi.ptgs.initFromConfigFile(cfg, arg_config_file_section.getValue());

    // ==================================================
    // ACTUAL PATH PLANNING
    // ==================================================
    const mpp::PlannerOutput plan = planner->plan(pi);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
              << " overall edges, " << plan.motionTree.nodes().size()
              << " nodes\n";

    if (!plan.bestNodeId.has_value())
    {
        std::cerr << "No bestNodeId in plan output.\n";
        return;
    }

    // backtrack:
    auto [plannedPath, pathEdges] =
        plan.motionTree.backtrack_path(*plan.bestNodeId);

    if (!arg_noRefine.isSet())
    {
        // refine:
        mpp::refine_trajectory(plannedPath, pathEdges, pi.ptgs);
    }

    // Visualize:
    mpp::VisualizationOptions vizOpts;

    vizOpts.renderOptions.highlight_path_to_node_id = plan.bestNodeId;
    vizOpts.renderOptions.color_normal_edge         = {0xb0b0b0, 0x20};  // RGBA

    vizOpts.renderOptions.showEdgeCosts = arg_showEdgeWeights.isSet();

    // Hide regular tree edges and only show best path?
    if (!arg_showTree.isSet()) vizOpts.renderOptions.width_normal_edge = 0;

    std::optional<mpp::trajectory_t> traj;  // interpolated path

    if (plan.success)
    {
        // generate path sequence:
        if (arg_printPathEdges.isSet())
        {
            std::cout << "Planned path edges:\n";
            for (const auto& edge : pathEdges) std::cout << edge->asString();
        }

        // interpolate path:
        if (arg_InterpolatePath.isSet() || arg_playAnimation.isSet())
        {
            const auto t0 = mrpt::Clock::nowDouble();

            traj = mpp::plan_to_trajectory(pathEdges, pi.ptgs);

            const auto dt = mrpt::Clock::nowDouble() - t0;

            std::cout << "Interpolated path done in "
                      << mrpt::system::intervalFormat(dt) << ".\n";

            if (arg_InterpolatePath.isSet())
            {
                std::cout << "Saving path to " << arg_InterpolatePath.getValue()
                          << std::endl;
                mpp::save_to_txt(*traj, arg_InterpolatePath.getValue());
            }
        }
    }

    // GUI:
    if (!arg_playAnimation.isSet() || !traj.has_value())
    {
        // regular UI:
        mpp::viz_nav_plan(plan, vizOpts, planner->costEvaluators_);
    }
    else
    {
        // Animation UI:
        mpp::viz_nav_plan_animation(
            plan, *traj, vizOpts.renderOptions, planner->costEvaluators_);
    }
}

int main(int argc, char** argv)
{
    try
    {
        bool cmdsOk = cmd.parse(argc, argv);

        if (argPlanner_yaml_output_file.isSet())
        {
            mpp::TPS_Astar_Parameters defaults;
            const auto                c = defaults.as_yaml();
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
