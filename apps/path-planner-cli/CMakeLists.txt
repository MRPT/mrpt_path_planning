project(path-planner-cli LANGUAGES CXX)

# find dependencies:
find_package(MRPT REQUIRED COMPONENTS gui nav tclap)

selfdriving_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES path-planner-cli.cpp
	LINK_LIBRARIES 
    mrpt::gui
    mrpt::nav
    mrpt::tclap
    mrpt_path_planning
)

