project(path-planner-cli LANGUAGES CXX)

# find dependencies:
find_package(mrpt_gui REQUIRED)
find_package(mrpt_nav REQUIRED)
find_package(CLI11 REQUIRED)

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    path-planner-cli.cpp
	LINK_LIBRARIES
    mrpt::mrpt_gui
    mrpt::mrpt_nav
    CLI11::CLI11
    mpp::mrpt_path_planning
    mrpt_path_planning_viz
)

