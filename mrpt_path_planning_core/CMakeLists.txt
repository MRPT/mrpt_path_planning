cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(default_build_type "Release")
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "SanitizeAddress" "SanitizeThread")

project(mrpt_path_planning_core LANGUAGES CXX)

include(CheckCXXCompilerFlag) # for check_cxx_compiler_flag()
include(CMakeDetermineCXXCompiler)

# Our custom scripts:
include(cmake/script_version_number.cmake REQUIRED)  #  Loads version number from package.xml
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
# NOTE: mrpt-gui and mvsim are intentionally NOT requested here, so this
# package (and everything that depends on it) has no GUI/display dependency.
# See mrpt_path_planning_apps for the CLI/GUI applications.
find_package(mrpt_containers REQUIRED)
find_package(mrpt_graphs REQUIRED)
find_package(mrpt_nav REQUIRED)
find_package(mrpt_maps REQUIRED)

# Explicit source/header lists (kept in sync by hand rather than globbed, so
# that adding/removing a file is a visible, reviewable change and reliably
# re-triggers CMake configure).
set(LIB_SRCS
	src/algos/bestTrajectory.cpp
	src/algos/CostEvaluator.cpp
	src/algos/CostEvaluatorCostMap.cpp
	src/algos/CostEvaluatorPreferredWaypoint.cpp
	src/algos/edge_interpolated_path.cpp
	src/algos/NavEngine.cpp
	src/algos/Planner.cpp
	src/algos/refine_trajectory.cpp
	src/algos/render_tree.cpp
	src/algos/render_vehicle.cpp
	src/algos/tp_obstacles_single_path.cpp
	src/algos/TPS_Astar.cpp
	src/algos/TPS_Astar_Bidir.cpp
	src/algos/trajectories.cpp
	src/algos/TrajectoryFollower.cpp
	src/algos/transform_pc_square_clipping.cpp
	src/algos/viz_svg.cpp
	src/data/MoveEdgeSE2_TPS.cpp
	src/data/PlannerInput.cpp
	src/data/PlannerOutput.cpp
	src/data/robot_shape_sampling.cpp
	src/data/SE2_KinState.cpp
	src/data/TrajectoriesAndRobotShape.cpp
	src/data/Waypoints.cpp
	src/interfaces/ObstacleSource.cpp
	src/interfaces/TargetApproachController.cpp
	src/interfaces/VehicleMotionInterface.cpp
	src/ptgs/DiffDrive_C.cpp
	src/ptgs/DiffDriveCollisionGridBased.cpp
	src/ptgs/HolonomicBlend.cpp
	src/registerClasses.cpp
)

set(LIB_PUBLIC_HDRS
	include/mpp/algos/bestTrajectory.h
	include/mpp/algos/CostEvaluator.h
	include/mpp/algos/CostEvaluatorCostMap.h
	include/mpp/algos/CostEvaluatorPreferredWaypoint.h
	include/mpp/algos/edge_interpolated_path.h
	include/mpp/algos/NavEngine.h
	include/mpp/algos/Planner.h
	include/mpp/algos/refine_trajectory.h
	include/mpp/algos/render_tree.h
	include/mpp/algos/render_vehicle.h
	include/mpp/algos/tp_obstacles_single_path.h
	include/mpp/algos/TPS_Astar.h
	include/mpp/algos/TPS_Astar_Bidir.h
	include/mpp/algos/trajectories.h
	include/mpp/algos/TrajectoryFollower.h
	include/mpp/algos/transform_pc_square_clipping.h
	include/mpp/algos/viz_svg.h
	include/mpp/algos/within_bbox.h
	include/mpp/data/basic_types.h
	include/mpp/data/const_ref_t.h
	include/mpp/data/EnqueuedMotionCmd.h
	include/mpp/data/MotionPrimitivesTree.h
	include/mpp/data/MoveEdgeSE2_TPS.h
	include/mpp/data/PlannerInput.h
	include/mpp/data/PlannerOutput.h
	include/mpp/data/ProgressCallbackData.h
	include/mpp/data/ptg_t.h
	include/mpp/data/RenderOptions.h
	include/mpp/data/robot_shape_sampling.h
	include/mpp/data/SampledTrajectory.h
	include/mpp/data/SE2_KinState.h
	include/mpp/data/TrajectoriesAndRobotShape.h
	include/mpp/data/Trajectory.h
	include/mpp/data/trajectory_t.h
	include/mpp/data/VehicleLocalizationState.h
	include/mpp/data/VehicleOdometryState.h
	include/mpp/data/Waypoints.h
	include/mpp/interfaces/LidarSource.h
	include/mpp/interfaces/ObstacleSource.h
	include/mpp/interfaces/TargetApproachController.h
	include/mpp/interfaces/TrajectoryVehicleInterface.h
	include/mpp/interfaces/VehicleMotionInterface.h
	include/mpp/ptgs/DiffDrive_C.h
	include/mpp/ptgs/DiffDriveCollisionGridBased.h
	include/mpp/ptgs/HolonomicBlend.h
	include/mpp/ptgs/SpeedTrimmablePTG.h
)

# define lib:
selfdriving_add_library(
	TARGET
		mrpt_path_planning
	SOURCES
		${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::mrpt_nav mrpt::mrpt_graphs mrpt::mrpt_containers mrpt::mrpt_maps
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
	CMAKE_DEPENDENCIES
		mrpt_containers mrpt_graphs mrpt_nav mrpt_maps
)

option(MRPT_PATH_PLANNING_BUILD_TESTS "Build mrpt_path_planning unit tests" ON)
if (MRPT_PATH_PLANNING_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()

# Summary:
message(STATUS "")
message(STATUS "${PROJECT_NAME} version ${MRPT_PATH_PLANNING_VERSION}")
message(STATUS "----------------------------------------------------------")
message(STATUS " CMAKE_BUILD_TYPE                : ${CMAKE_BUILD_TYPE}")
message(STATUS " MRPT                            : ${mrpt-nav_VERSION}")
message(STATUS "")

