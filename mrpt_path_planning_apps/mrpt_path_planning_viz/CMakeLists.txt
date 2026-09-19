project(mrpt_path_planning_viz LANGUAGES CXX)

# GUI helpers built on top of mrpt_path_planning_core (opens a real
# mrpt::gui::CDisplayWindow3D window). Kept as its own small library, rather
# than folded into path-planner-cli, so external consumers (e.g. ROS nodes)
# that call mpp::viz_nav_plan() can link it directly.
selfdriving_add_library(
	TARGET
		mrpt_path_planning_viz
	SOURCES
		include/mpp/algos/viz.h
		src/algos/viz.cpp
	PUBLIC_LINK_LIBRARIES
		mrpt::mrpt_gui
		mpp::mrpt_path_planning
	CMAKE_DEPENDENCIES
		mrpt_gui
		mrpt_path_planning
)
