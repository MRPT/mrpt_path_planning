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

	const auto obs_pts = in.obstacles->obstacles();
	ASSERT_(obs_pts);
	float min_x, max_x, min_y, max_y, min_z, max_z;
	obs_pts->boundingBox(min_x, max_x, min_y, max_y, min_z, max_z);

	// Build an occ. grid: all free except obstacles in the input
	// pointcloud.
	mrpt::maps::COccupancyGridMap2D obsGrid;
	obsGrid.setSize(
		min_x, max_x, min_y, max_y, static_cast<float>(params_.grid_resolution),
		0.99f);

	std::deque<mrpt::math::TPoint2D> path;
	bool                             path_not_found;

	grid_planner.computePath(
		obsGrid, mrpt::poses::CPose2D(in.state_start.pose),
		mrpt::poses::CPose2D(in.state_goal.pose), path, path_not_found);

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
