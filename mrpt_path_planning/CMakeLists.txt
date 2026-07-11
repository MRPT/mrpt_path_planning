project(mrpt_path_planning LANGUAGES CXX)

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
	src/algos/transform_pc_square_clipping.cpp
	src/algos/viz.cpp
	src/algos/viz_svg.cpp
	src/data/MoveEdgeSE2_TPS.cpp
	src/data/PlannerInput.cpp
	src/data/PlannerOutput.cpp
	src/data/SE2_KinState.cpp
	src/data/TrajectoriesAndRobotShape.cpp
	src/data/Waypoints.cpp
	src/follow/algos/TrajectoryFollower.cpp
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
	include/mpp/algos/transform_pc_square_clipping.h
	include/mpp/algos/viz.h
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
	include/mpp/data/SE2_KinState.h
	include/mpp/data/TrajectoriesAndRobotShape.h
	include/mpp/data/trajectory_t.h
	include/mpp/data/VehicleLocalizationState.h
	include/mpp/data/VehicleOdometryState.h
	include/mpp/data/Waypoints.h
	include/mpp/follow/algos/TrajectoryFollower.h
	include/mpp/follow/data/SampledTrajectory.h
	include/mpp/follow/data/Trajectory.h
	include/mpp/follow/interfaces/TrajectoryVehicleInterface.h
	include/mpp/interfaces/LidarSource.h
	include/mpp/interfaces/MVSIM_VehicleInterface.h
	include/mpp/interfaces/ObstacleSource.h
	include/mpp/interfaces/TargetApproachController.h
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
		mrpt::nav mrpt::graphs mrpt::containers mrpt::gui mrpt::maps
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
	CMAKE_DEPENDENCIES 
		mrpt-containers mrpt-graphs mrpt-gui mrpt-nav
)

