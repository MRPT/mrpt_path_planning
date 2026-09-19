/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mpp/algos/CostEvaluatorCostMap.h>
#include <mpp/algos/CostEvaluatorPreferredWaypoint.h>
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/refine_trajectory.h>
#include <mpp/algos/trajectories.h>
#include <mpp/algos/viz.h>
#include <mpp/algos/viz_svg.h>
#include <mpp/data/Waypoints.h>
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

#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>

// CLI globals (populated in main before helper functions are called):
static std::string  arg_obs_file;
static std::string  argPlanner{"mpp::TPS_Astar"};
static std::string  argVerbosity{"INFO"};
static float        argObstaclesGridResolution{0.05f};
static float        arg_interpolation_period{0.25f};
static std::string  arg_ptgs_file;
static std::string  argPlanner_yaml_file;
static std::string  argPlanner_yaml_output_file;
static std::string  arg_config_file_section{"SelfDriving"};
static std::string  arg_start_pose;
static std::string  arg_start_vel{"[0 0 0]"};
static bool         arg_start_vel_set{false};
static std::string  arg_goal_pose{"[0 0 0]"};
static double       argBBoxMargin{2.0};
static std::string  arg_goal_vel;
static bool         arg_goal_vel_set{false};
static unsigned int argRandomSeed{0};
static bool         argRandomSeed_set{false};
static std::string  arg_plugins;
static std::string  arg_costMap;
static bool         arg_costMap_set{false};
static std::string  arg_waypoints;
static bool         arg_waypoints_set{false};
static std::string  arg_waypointsParams;
static bool         arg_waypointsParams_set{false};
static bool         arg_showTree{false};
static bool         arg_ignoreObstaclesBbox{false};
static bool         arg_noRefine{false};
static bool         arg_showEdgeWeights{false};
static bool         arg_printPathEdges{false};
static std::string  arg_InterpolatePath;
static bool         arg_InterpolatePath_set{false};
static std::string  arg_save_svg;
static bool         arg_save_svg_set{false};
static bool         arg_playAnimation{false};
static bool         arg_noGui{false};
static size_t       arg_svg_tree_decimation{1};
static bool         arg_svg_no_tree{false};

static mrpt::maps::CPointsMap::Ptr load_obstacles()
{
    auto obsPts = mrpt::maps::CSimplePointsMap::Create();

    const auto& sFile = arg_obs_file;
    ASSERT_FILE_EXISTS_(sFile);

    const auto sExt =
        mrpt::system::extractFileExtension(sFile, true /*ignore .gz*/);

    if (mrpt::system::strCmpI(sExt, "txt") ||
        mrpt::system::strCmpI(sExt, "pts"))
    {
        if (!obsPts->load2D_from_text_file(sFile))
        {
            THROW_EXCEPTION_FMT(
                "Cannot read obstacle point cloud from: `%s`",
                arg_obs_file.c_str());
        }
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
        grid.loadFromBitmapFile(sFile, argObstaclesGridResolution);
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

    pi.stateStart.pose.fromString(arg_start_pose);
    if (arg_start_vel_set) { pi.stateStart.vel.fromString(arg_start_vel); }

    pi.stateGoal.state = mpp::PoseOrPoint::FromString(arg_goal_pose);
    if (arg_goal_vel_set) { pi.stateGoal.vel.fromString(arg_goal_vel); }

    pi.obstacles.emplace_back(obs);

    mrpt::math::TBoundingBoxf bbox;

    if (!arg_ignoreObstaclesBbox) { bbox = obs->obstacles()->boundingBox(); }
    else
    {
        // This will ensure the next updateWithPoint() will set the bbox:
        bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
    }

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin =
            mrpt::math::TPoint3Df(argBBoxMargin, argBBoxMargin, .0);
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

    // Do the path planning:
    mpp::Planner::Ptr planner = std::dynamic_pointer_cast<mpp::Planner>(
        mrpt::rtti::classFactory(argPlanner));

    if (!planner)
    {
        THROW_EXCEPTION_FMT(
            "Given classname '%s' does not seem to be a known C++ class "
            "implementing `Planner",
            argPlanner.c_str());
    }

    // Enable time profiler:
    planner->profiler_().enable(true);

    // PTGs config file (loaded before the costmap so the robot shape is
    // available to make the costmap footprint-aware):
    std::cout << "[PTGs] Initializing PTGs..." << std::endl;

    mrpt::config::CConfigFile cfg(arg_ptgs_file);
    pi.ptgs.initFromConfigFile(cfg, arg_config_file_section);

    std::cout << "[PTGs] Done." << std::endl;

    if (arg_costMap_set)
    {
        // cost map:
        const auto costMapParams =
            mpp::CostEvaluatorCostMap::Parameters::FromYAML(
                mrpt::containers::yaml::FromFile(arg_costMap));

        auto costmap = mpp::CostEvaluatorCostMap::FromStaticPointObstacles(
            *obsPts, costMapParams, pi.stateStart.pose, pi.ptgs.robotShape);

        planner->costEvaluators_.push_back(costmap);
    }

    // Preferred waypoints:
    auto wpParams = mpp::CostEvaluatorPreferredWaypoint::Parameters();
    if (arg_waypointsParams_set)
    {
        wpParams = mpp::CostEvaluatorPreferredWaypoint::Parameters::FromYAML(
            mrpt::containers::yaml::FromFile(arg_waypointsParams));
    }

    if (arg_waypoints_set)
    {
        const auto wps = mpp::WaypointSequence::FromYAML(
            mrpt::containers::yaml::FromFile(arg_waypoints));

        std::vector<mrpt::math::TPoint2D> lstPts;
        for (const auto& wp : wps.waypoints) { lstPts.emplace_back(wp.target); }

        auto costEval     = mpp::CostEvaluatorPreferredWaypoint::Create();
        costEval->params_ = wpParams;
        costEval->setPreferredWaypoints(lstPts);
        planner->costEvaluators_.push_back(costEval);
    }

    // verbosity level:
    planner->setMinLoggingLevel(
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
            argVerbosity));

    // Set planner required params:
    if (!argPlanner_yaml_file.empty())
    {
        const auto& sFile = argPlanner_yaml_file;
        ASSERT_FILE_EXISTS_(sFile);
        const auto c = mrpt::containers::yaml::FromFile(sFile);
        planner->params_from_yaml(c);
        std::cout << "Loaded these planner params:\n";
        planner->params_as_yaml().printAsYAML();
    }

    // Insert custom progress callback:
    planner->progressCallback_ = [](const mpp::ProgressCallbackData& pcd)
    {
        std::cout << "[progressCallback] bestCostFromStart: "
                  << pcd.bestCostFromStart
                  << " bestCostToGoal: " << pcd.bestCostToGoal
                  << " bestPathLength: " << pcd.bestPath.size() << std::endl;
    };

    // ==================================================
    // ACTUAL PATH PLANNING
    // ==================================================
    const mpp::PlannerOutput plan = planner->plan(pi);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.motionTree.edges_to_children.size()
              << " overall edges, " << plan.motionTree.nodes().size()
              << " nodes\n";

    if (arg_save_svg_set)
    {
        mpp::SvgExportOptions svgOpts;
        svgOpts.draw_tree       = !arg_svg_no_tree;
        svgOpts.tree_decimation = arg_svg_tree_decimation;
        if (mpp::save_plan_to_svg(plan, arg_save_svg, svgOpts))
        {
            std::cout << "Saved SVG plot: " << arg_save_svg << "\n";
        }
        else { std::cerr << "Could not write SVG: " << arg_save_svg << "\n"; }
    }

    if (!plan.bestNodeId.has_value())
    {
        std::cerr << "No bestNodeId in plan output.\n";
        return;
    }

    // backtrack:
    auto [plannedPath, pathEdges] =
        plan.motionTree.backtrack_path(*plan.bestNodeId);

    if (!arg_noRefine)
    {
        // refine:
        mpp::refine_trajectory(plannedPath, pathEdges, pi.ptgs);
    }

    // Visualize:
    mpp::VisualizationOptions vizOpts;

    vizOpts.renderOptions.highlight_path_to_node_id = plan.bestNodeId;
    vizOpts.renderOptions.color_normal_edge         = {0xb0b0b0, 0x20};  // RGBA

    vizOpts.renderOptions.showEdgeCosts = arg_showEdgeWeights;

    // Hide regular tree edges and only show best path?
    if (!arg_showTree) { vizOpts.renderOptions.width_normal_edge = 0; }

    std::optional<mpp::trajectory_t> traj;  // interpolated path

    if (plan.success)
    {
        // generate path sequence:
        if (arg_printPathEdges)
        {
            std::cout << "Planned path edges:\n";
            for (const auto& edge : pathEdges)
            {
                std::cout << edge->asString();
            }
        }

        // interpolate path:
        if (arg_InterpolatePath_set || arg_playAnimation)
        {
            const auto t0 = mrpt::Clock::nowDouble();

            const double interpPeriod = arg_interpolation_period;

            traj = mpp::plan_to_trajectory(pathEdges, pi.ptgs, interpPeriod);

            // Note: trajectory is in local frame of reference
            // of plan.originalInput.stateStart.pose
            // so, correct that relative pose so we keep everything in global
            // frame:
            const auto& startPose = plan.originalInput.stateStart.pose;
            for (auto& kv : *traj)
            {
                kv.second.state.pose = startPose + kv.second.state.pose;
            }

            const auto dt = mrpt::Clock::nowDouble() - t0;

            std::cout << "Interpolated path done in "
                      << mrpt::system::intervalFormat(dt) << ".\n";

            if (arg_InterpolatePath_set)
            {
                std::cout << "Saving path to " << arg_InterpolatePath
                          << std::endl;
                mpp::save_to_txt(*traj, arg_InterpolatePath);
            }
        }
    }

    // GUI (skipped entirely in headless/batch mode):
    if (arg_noGui) return;
    if (!arg_playAnimation || !traj.has_value())
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
        CLI::App app{"path-planner-cli"};

        app.add_option(
               "-o,--obstacles", arg_obs_file,
               "Input obstacles: either (1) a .txt file with obstacle points "
               "(one 'x y' "
               "pair per line), or (2) a .gridmap file, or (3) a ROS MAP YAML "
               "file, or a "
               "(4) png file with an gray-scale occupancy grid (*.png, *.bmp)")
            ->required();
        app.add_option(
            "-p,--planner", argPlanner, "Planner C++ class name to use.");
        app.add_option(
            "-v,--verbose", argVerbosity, "Verbosity level for path planner.");
        app.add_option(
            "--obstacles-gridimage-resolution", argObstaclesGridResolution,
            "Only if --obstacles points to an image file, this sets the length "
            "of each pixel in "
            "meters.");
        app.add_option(
            "--interpolarion-period", arg_interpolation_period,
            "Interpolation (and animation) time step between keyframes [s].");
        app.add_option(
               "-c,--ptg-config", arg_ptgs_file,
               "Input .ini file with PTG definitions.")
            ->required();
        app.add_option(
            "--planner-parameters", argPlanner_yaml_file,
            "Input .yaml file with planner parameters.");
        app.add_option(
            "--write-planner-parameters", argPlanner_yaml_output_file,
            "If defined, it will save default planner params to a .yaml file "
            "and exit.");
        app.add_option(
            "--config-section", arg_config_file_section,
            "If loading from an INI file, the name of the section to load.");
        app.add_option("-s,--start-pose", arg_start_pose, "Start 2D pose.")
            ->required();
        app.add_option("--start-vel", arg_start_vel, "Start 2D velocity.");
        app.add_option(
            "-g,--goal-pose", arg_goal_pose, "Goal 2D pose or point.");
        app.add_option(
            "--bbox-margin", argBBoxMargin,
            "Margin to add to the start-goal bbox poses.");
        app.add_option("--goal-vel", arg_goal_vel, "Goal 2D velocity.");
        app.add_option(
            "--random-seed", argRandomSeed,
            "Pseudorandom generator seed (default: from time).");
        app.add_option(
            "--plugins", arg_plugins,
            "Optional plug-in libraries to load, for externally-defined PTGs.");
        app.add_option(
            "--costmap-obstacles", arg_costMap,
            "Creates a costmap from obstacle point clouds with the given "
            "parameters from a YAML "
            "file.");
        app.add_option(
            "--waypoints", arg_waypoints,
            "This creates a preferred-waypoints costlayer, from the waypoint "
            "list in a given "
            "YAML file.");
        app.add_option(
            "--waypoints-parameters", arg_waypointsParams,
            "If --waypoints is also set, this loads the preferred waypoints "
            "costlayer "
            "parameters from a YAML file.");
        app.add_flag(
            "--show-tree", arg_showTree,
            "Shows the whole search tree instead of just the best path.");
        app.add_flag(
            "--ignore-obstacles-bbox", arg_ignoreObstaclesBbox,
            "Ignore obstacles for estimating the problem world bounding box.");
        app.add_flag(
            "--no-refine", arg_noRefine, "Skips the post-plan refine stage.");
        app.add_flag(
            "--show-edge-weights", arg_showEdgeWeights,
            "Shows the weight of path edges.");
        app.add_flag(
            "--print-path-edges", arg_printPathEdges,
            "Prints details on the found planned path edges.");
        app.add_option(
            "--save-interpolated-path", arg_InterpolatePath,
            "Interpolates the path and saves it into a .csv file.");
        app.add_option(
            "--save-svg", arg_save_svg,
            "Saves a 2D SVG plot of the plan (obstacles, tree, path, robot "
            "shapes) for "
            "debugging / paper figures.");
        app.add_flag(
            "--play-animation", arg_playAnimation,
            "Shows the GUI with an animation of the vehicle moving along the "
            "path.");
        app.add_flag(
            "--no-gui", arg_noGui,
            "Do not open any GUI window (for headless/batch use, e.g. mass "
            "figure "
            "export). The process exits without waiting for a window to be "
            "closed.");
        app.add_option(
            "--svg-tree-decimation", arg_svg_tree_decimation,
            "When exporting SVG, draw only one motion-tree edge out of every "
            "N. Use a "
            "larger value to thin out very dense (e.g. failed-query) trees.");
        app.add_flag(
            "--svg-no-tree", arg_svg_no_tree,
            "When exporting SVG, omit the motion tree entirely.");

        CLI11_PARSE(app, argc, argv);

        // Detect which optional string options were actually provided:
        arg_start_vel_set       = (app.count("--start-vel") > 0);
        arg_goal_vel_set        = (app.count("--goal-vel") > 0);
        argRandomSeed_set       = (app.count("--random-seed") > 0);
        arg_costMap_set         = (app.count("--costmap-obstacles") > 0);
        arg_waypoints_set       = (app.count("--waypoints") > 0);
        arg_waypointsParams_set = (app.count("--waypoints-parameters") > 0);
        arg_InterpolatePath_set = (app.count("--save-interpolated-path") > 0);
        arg_save_svg_set        = (app.count("--save-svg") > 0);

        if (!argPlanner_yaml_output_file.empty())
        {
            mpp::TPS_Astar_Parameters defaults;
            const auto                c     = defaults.as_yaml();
            const auto&               sFile = argPlanner_yaml_output_file;
            std::ofstream             fileYaml(sFile);
            ASSERT_(fileYaml.is_open());
            c.printAsYAML(fileYaml);
            std::cout << "Wrote file: " << sFile << std::endl;
            return 0;
        }

        if (!arg_plugins.empty())
        {
            std::string loadErrors;
            if (!mrpt::system::loadPluginModules(arg_plugins, loadErrors))
            {
                std::cerr << "Could not load plugins, error: " << loadErrors;
                return 1;
            }
        }

        if (argRandomSeed_set)
        {
            mrpt::random::getRandomGenerator().randomize(argRandomSeed);
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
