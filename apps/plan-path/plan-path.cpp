/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/Planner_Astar.h>
#include <libselfdriving/viz.h>
#include <mrpt/config/CConfigFile.h>
#include <tclap/CmdLine.h>
#include <iostream>

#if MRPT_VERSION >= 0x199
#include <mrpt/core/exceptions.h>  // exception_to_str()
#endif

static TCLAP::CmdLine cmd("plan-path");

static TCLAP::ValueArg<std::string> arg_obs_file(
    "o", "obstacles", "Input .txt file with obstacle points.", true, "",
    "obs.txt", cmd);

static TCLAP::ValueArg<std::string> arg_ptgs_file(
    "p", "ptg-config", "Input .ini file with PTG definitions.", true, "",
    "ptgs.ini", cmd);

static TCLAP::ValueArg<std::string> arg_start_pose(
    "s", "start-pose", "Start 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

static TCLAP::ValueArg<std::string> arg_goal_pose(
    "g", "goal-pose", "Goal 2D pose", true, "", "\"[x y phi_deg]\"", cmd);

static void do_plan_path()
{
    // Load obstacles:
    mrpt::maps::CSimplePointsMap obsPts;
    if (!obsPts.load2D_from_text_file(arg_obs_file.getValue()))
        THROW_EXCEPTION_FMT(
            "Cannot read obstacle point cloud from: `%s`",
            arg_obs_file.getValue().c_str());

    auto obs = std::make_shared<selfdrive::ObstacleSource>(obsPts);

    // Prepare planner input data:
    selfdrive::PlannerInput pi;

    pi.state_start.pose.fromString(arg_start_pose.getValue());
    pi.state_goal.pose.fromString(arg_goal_pose.getValue());
    pi.obstacles                = obs;
    pi.robot_shape.robot_radius = 0.35;

    std::cout << "Start pose: " << pi.state_start.pose.asString() << "\n";
    std::cout << "Goal pose : " << pi.state_goal.pose.asString() << "\n";
    std::cout << "Obstacles : " << pi.obstacles->obstacles()->size()
              << " points\n";

    // Do the path planning :
    selfdrive::Planner_Astar planner;

    // Enable time profiler:
    planner.profiler_.enable(true);

    // Set planner required params:
    planner.params_.grid_resolution = 0.05;

    // PTGs config file:
    mrpt::config::CConfigFile cfg(arg_ptgs_file.getValue());
    pi.ptgs.initFromConfigFile(cfg, "CReactiveNavigationSystem");

    const selfdrive::NavPlan plan = planner.plan(pi);

    std::cout << "\nDone.\n";
    std::cout << "Success: " << (plan.success ? "YES" : "NO") << "\n";
    std::cout << "Plan has " << plan.actions.size() << " steps\n";

    // Visualize:
    selfdrive::viz_nav_plan(plan);
}

int main(int argc, char** argv)
{
    try
    {
        cmd.parse(argc, argv);
        do_plan_path();
        return 0;
    }
    catch (std::exception& e)
    {
#if MRPT_VERSION >= 0x199
        std::cerr << mrpt::exception_to_str(e);
#else
        std::cerr << e.what();
#endif
        return 1;
    }
}
