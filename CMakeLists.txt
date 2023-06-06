cmake_minimum_required(VERSION 3.1)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(default_build_type "Release")
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "SanitizeAddress" "SanitizeThread")

project(MRPTPathPlanningProject LANGUAGES CXX)

# Include std cmake scripts:
include(CheckCXXCompilerFlag) # for check_cxx_compiler_flag()
include(CMakeDetermineCXXCompiler)

# Our custom scripts:
include(cmake/script_version_number.cmake REQUIRED)	#  Loads version number from package.xml
include(cmake/misc_cmake_functions.cmake REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(CMAKE_COMPILER_IS_GNUCXX)
	# BUILD_TYPE: SanitizeAddress
	set(CMAKE_CXX_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak -g")
	set(CMAKE_EXE_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")
	set(CMAKE_SHARED_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")
	
	# BUILD_TYPE: SanitizeThread
	set(CMAKE_CXX_FLAGS_SANITIZETHREAD "-fsanitize=thread -g")
	set(CMAKE_EXE_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
	set(CMAKE_SHARED_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
endif()

# Dependencies:
find_package(mvsim-simulator QUIET)
find_package(mrpt-tclap REQUIRED)
find_package(mrpt-containers REQUIRED)
find_package(mrpt-graphs REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-nav REQUIRED)

# Targets:
add_subdirectory(mrpt_path_planning)

option(MRPT_PATH_PLANNING_BUILD_APPS "Build mrpt_path_planning cli/gui tools" ON)
if (MRPT_PATH_PLANNING_BUILD_APPS)
	add_subdirectory(apps)
endif()

# Summary:
message(STATUS "")
message(STATUS "${PROJECT_NAME} version ${MRPT_PATH_PLANNING_VERSION}")
message(STATUS "----------------------------------------------------------")
message(STATUS " CMAKE_BUILD_TYPE                : ${CMAKE_BUILD_TYPE}")
message(STATUS " MRPT                            : ${mrpt-nav_VERSION}")
message(STATUS " MVSIM                           : ${mvsim-simulator_FOUND} (version ${mvsim-simulator_VERSION})")
message(STATUS "")
