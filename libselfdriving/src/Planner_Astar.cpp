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

    ret.state_start = in.state_start;
    ret.state_goal  = in.state_goal;

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

#if 1
    obsGrid.saveAsBitmapFile("gridmap.png");
#endif

    // Run A* itself:
    profiler_.enter("plan.computePath");

    std::deque<mrpt::math::TPoint2D> path;
    bool                             path_not_found;

    grid_planner.computePath(
        obsGrid, mrpt::poses::CPose2D(in.state_start.pose),
        mrpt::poses::CPose2D(in.state_goal.pose), path, path_not_found);

    profiler_.leave("plan.computePath");

    if (path_not_found)
    {
        ret.success = false;
        return ret;
    }

    // Go thru the list of points and convert them into a sequence of PTG
    // actions:

    return ret;
    MRPT_END
}
