# ------------------------------------------------------------------------------
#  mrpt_path_planning metapackage
#
#  Kept for backward compatibility: depends on mrpt_path_planning_core and
#  mrpt_path_planning_apps, so existing <depend>mrpt_path_planning</depend>
#  consumers keep building/installing the exact same set of libraries and
#  applications as before.
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22...3.31)

project(mrpt_path_planning NONE)

# This is a metapackage: there are no targets, no sources to build. The
# install() call below has nothing to copy; it only exists so CMake
# generates an "install" target (colcon otherwise warns that there's none).
install(DIRECTORY DESTINATION share/${PROJECT_NAME})
