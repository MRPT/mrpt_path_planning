cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(default_build_type "Release")
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "SanitizeAddress" "SanitizeThread")

project(mrpt_path_planning_apps LANGUAGES CXX)

# Extract version from package.xml (only major/minor/patch are needed here;
# no generated header, unlike mrpt_path_planning_core):
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${CMAKE_MATCH_1})
set(SELFDRIVING_MAJOR_VERSION ${CMAKE_MATCH_1})
set(SELFDRIVING_MINOR_VERSION ${CMAKE_MATCH_2})
set(SELFDRIVING_PATCH_VERSION ${CMAKE_MATCH_3})

# Our custom scripts:
include(cmake/misc_cmake_functions.cmake REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Dependencies: the headless algorithms library (sibling package, built and
# installed first by colcon), plus GUI/simulator libs that mrpt_path_planning_core
# deliberately does not depend on.
find_package(mrpt_path_planning REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-nav REQUIRED)
find_package(mrpt-tclap REQUIRED)

# mvsim is optional: selfdriving-simulator-gui silently skips itself if absent.
find_package(mvsim-simulator QUIET)
find_package(mvsim-comms QUIET)

# Targets:
add_subdirectory(path-planner-cli)
add_subdirectory(selfdriving-simulator-gui)

# Install example/demo config files (PTG .ini, planner .yaml, obstacle maps):
install(DIRECTORY share/ DESTINATION share/${PROJECT_NAME})

# Summary:
message(STATUS "")
message(STATUS "${PROJECT_NAME} version ${SELFDRIVING_MAJOR_VERSION}.${SELFDRIVING_MINOR_VERSION}.${SELFDRIVING_PATCH_VERSION}")
message(STATUS "----------------------------------------------------------")
message(STATUS " CMAKE_BUILD_TYPE : ${CMAKE_BUILD_TYPE}")
message(STATUS " MVSIM            : ${mvsim-simulator_FOUND} (version ${mvsim-simulator_VERSION})")
message(STATUS "")
