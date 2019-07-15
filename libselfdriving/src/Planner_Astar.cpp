/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <libselfdriving/Planner_Astar.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <iostream>

using namespace selfdrive;

NavPlan Planner_Astar::plan(const PlannerInput& in)
{
    MRPT_START
    CTimeLoggerEntry tleg(profiler_, "plan");

    NavPlan ret;

    ret.original_input = in;

    // Reuse MRPT's implementation:
    mrpt::nav::PlannerSimple2D grid_planner;

    ASSERT_ABOVE_(in.robot_shape.robot_radius, .0);
    grid_planner.robotRadius = static_cast<float>(in.robot_shape.robot_radius);
    grid_planner.minStepInReturnedPath = static_cast<float>(in.min_step_len);

    // Get obstacles:
    ASSERT_(in.obstacles);
    if (in.obstacles->dynamic())
        std::cerr << "[Planner_Astar] Warning: Ignoring dynamic obstacles.\n";

    profiler_.enter("plan.obstacles()");
    const auto obs_pts = in.obstacles->obstacles();
    profiler_.leave("plan.obstacles()");

    ASSERT_(obs_pts);
    float min_x, max_x, min_y, max_y, min_z, max_z;
    obs_pts->boundingBox(min_x, max_x, min_y, max_y, min_z, max_z);

    // make sure the bbox includes the goal & start poses as well as obstacles:
    mrpt::keep_max(max_x, in.state_start.pose.x + 1.0);
    mrpt::keep_max(max_x, in.state_goal.pose.x + 1.0);
    mrpt::keep_max(max_y, in.state_start.pose.y + 1.0);
    mrpt::keep_max(max_y, in.state_goal.pose.y + 1.0);

    mrpt::keep_min(min_x, in.state_start.pose.x - 1.0);
    mrpt::keep_min(min_x, in.state_goal.pose.x - 1.0);
    mrpt::keep_min(min_y, in.state_start.pose.y - 1.0);
    mrpt::keep_min(min_y, in.state_goal.pose.y - 1.0);

    // Build an occ. grid: all free except obstacles in the input
    // pointcloud.
    profiler_.enter("plan.build_grid");

    mrpt::maps::COccupancyGridMap2D obsGrid;
    obsGrid.setSize(
        min_x, max_x, min_y, max_y, static_cast<float>(params_.grid_resolution),
        0.99f);
    {
        const auto& xs = obs_pts->getPointsBufferRef_x();
        const auto& ys = obs_pts->getPointsBufferRef_y();

        for (std::size_t i = 0; i < xs.size(); i++)
            obsGrid.setPos(xs[i], ys[i], 0.01f /* low "freeness" */);
    }
    profiler_.leave("plan.build_grid");

    // Run A* itself:
    profiler_.enter("plan.computePath");

    std::deque<mrpt::math::TPoint2D> path;
    bool                             path_not_found;

    grid_planner.computePath(
        obsGrid, mrpt::poses::CPose2D(in.state_start.pose),
        mrpt::poses::CPose2D(in.state_goal.pose), path, path_not_found);

    profiler_.leave("plan.computePath");

    if (path_not_found || path.empty())
    {
        ret.success = false;
        return ret;
    }

    // Go thru the list of points and convert them into a sequence of PTG
    // actions:
    SE2_KinState last_state;
    last_state = in.state_start;

    // Make sure PTGs are initialized
    if (!in.ptgs.ptgs.empty())
    {
        CTimeLoggerEntry tle(profiler_, "plan.init_PTGs");
        for (auto& ptg : in.ptgs.ptgs)
        {
            ASSERT_(ptg);
            ptg->initialize();
        }
    }

    for (const auto& p : path)
    {
        NavPlanAction act;
        act.state_from    = last_state;
        act.state_to.pose = mrpt::math::TPose2D(p.x, p.y, 0);

        // Compute PTG actions (trajectory segments):
        if (!in.ptgs.ptgs.empty())
        {
            //
            MRPT_TODO("adjust phi() according to ptg");
        }

        // for the next iter:
        last_state = act.state_to;

        ret.actions.push_back(std::move(act));
    }

    return ret;
    MRPT_END
}
