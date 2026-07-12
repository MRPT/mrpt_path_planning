project(path-planner-cli LANGUAGES CXX)

# find dependencies:
find_package(MRPT REQUIRED COMPONENTS gui nav tclap)

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    path-planner-cli.cpp
    viz.cpp
    viz.h
	LINK_LIBRARIES
    mrpt::gui
    mrpt::nav
    mrpt::tclap
    mpp::mrpt_path_planning
)

