# mrpt_path_planning: Architecture & Theoretical Foundations

## 1. Project Overview

**mrpt_path_planning** (namespace: `mpp`) is a C++17 library for kinematically-feasible path planning on planar environments with point-cloud obstacles. It builds on the MRPT libraries and uses **Parameterized Trajectory Generators (PTGs)** as motion primitives to plan paths for vehicles with realistic kinematics (differential-drive, Ackermann, holonomic).

- **License**: BSD
- **Build**: CMake (standalone) or colcon (ROS 2)
- **Dependencies**: MRPT >= 2.12.0 (mrpt-nav, mrpt-maps, mrpt-graphs, mrpt-gui, mrpt-containers, mrpt-tclap)

## 2. Directory Structure

```
mrpt_path_planning/
├── mrpt_path_planning/           # Core library
│   ├── include/mpp/
│   │   ├── algos/                # Planning algorithms
│   │   │   ├── TPS_Astar.h       # ★ Main A* planner
│   │   │   ├── Planner.h         # Abstract planner base
│   │   │   ├── CostEvaluator.h/CostEvaluatorCostMap.h/CostEvaluatorPreferredWaypoint.h
│   │   │   ├── tp_obstacles_single_path.h  # Collision checking
│   │   │   ├── edge_interpolated_path.h    # Path interpolation
│   │   │   ├── transform_pc_square_clipping.h  # Obstacle transform
│   │   │   ├── render_tree.h / render_vehicle.h / viz.h  # Visualization
│   │   │   ├── bestTrajectory.h / trajectories.h / refine_trajectory.h
│   │   │   └── within_bbox.h / NavEngine.h
│   │   ├── data/                 # Data structures
│   │   │   ├── SE2_KinState.h    # SE(2) pose+velocity state
│   │   │   ├── MotionPrimitivesTree.h  # Motion tree (graph)
│   │   │   ├── MoveEdgeSE2_TPS.h # Edge: PTG-based motion segment
│   │   │   ├── PlannerInput.h / PlannerOutput.h
│   │   │   ├── trajectory_t.h / Waypoints.h / TrajectoriesAndRobotShape.h
│   │   │   ├── basic_types.h / ptg_t.h / RenderOptions.h
│   │   │   ├── EnqueuedMotionCmd.h / ProgressCallbackData.h
│   │   │   └── VehicleLocalizationState.h / VehicleOdometryState.h
│   │   ├── interfaces/           # Abstract interfaces
│   │   │   ├── ObstacleSource.h  # Obstacle providers
│   │   │   ├── VehicleMotionInterface.h  # Vehicle control
│   │   │   └── TargetApproachController.h
│   │   └── ptgs/                 # PTG implementations
│   │       ├── SpeedTrimmablePTG.h
│   │       ├── DiffDriveCollisionGridBased.h
│   │       ├── DiffDrive_C.h     # Circular-arc PTG
│   │       └── HolonomicBlend.h  # Holonomic blend PTG
│   └── src/                      # Corresponding .cpp files
├── apps/
│   ├── path-planner-cli/         # CLI tool for planning
│   └── selfdriving-simulator-gui/ # GUI simulator (requires mvsim)
├── share/                        # Config files: PTG .ini, planner .yaml, obstacles .txt
├── wip-experimental/             # TPS_RRTstar (work-in-progress)
└── package.xml                   # ROS 2 package manifest
```

## 3. Theoretical Foundations

### 3.1 Parameterized Trajectory Generators (PTGs)

PTGs are the core motion primitive abstraction (from MRPT's `mrpt::nav`). A PTG defines a **family of trajectories** parameterized by:

- **Trajectory index `k`** (or equivalently an angle `alpha` in [-π, π]): selects which trajectory in the family.
- **Distance `d`** (in "pseudometers"): how far along the trajectory to travel.
- **Speed** (for speed-trimmable PTGs): a [0,1] scaling factor on velocity.

Each PTG maps:
- **(k, step)** → **(x, y, phi)**: the pose at a given step along trajectory k.
- **(k, step)** → **(vx, vy, omega)**: the velocity twist at that point.
- **(x, y)** → **(k, d)**: inverse mapping from workspace to TP-Space (`inverseMap_WS2TP`).

PTGs used in this project:
- **`DiffDrive_C`**: Circular arcs for differential-drive / Ackermann. Each `k` selects a constant turning radius.
- **`HolonomicBlend`**: For holonomic robots with velocity ramping. Uses runtime-compiled expressions for velocity profiles.

**TP-Space obstacle mapping**: Given obstacle points in the robot's local frame, `updateTPObstacleSingle(ox, oy, k, d_free)` computes the free distance along trajectory `k` before collision. This is the key to efficient collision checking — obstacles are projected from workspace to TP-Space.

### 3.2 SE(2) Lattice-Based A*

The planner (`TPS_Astar`) discretizes the configuration space SE(2) = R² × S¹ into a grid lattice:

- **xy cells**: `grid_resolution_xy` (default 0.20 m)
- **yaw cells**: `grid_resolution_yaw` (default 5°)

Each lattice cell stores an A* `Node` with:
- Exact SE(2) state (pose + velocity), not snapped to cell center
- g-score (cost from start), f-score (g + heuristic)
- Parent pointer for path reconstruction

**Key difference from classical A***: Neighbors are not fixed grid adjacencies. Instead, the planner generates neighbors dynamically by forward-simulating PTG trajectories from the current node, sampling:
- A subset of trajectory indices `k` (up to `max_ptg_trajectories_to_explore`)
- Multiple time horizons (`ptg_sample_timestamps`, e.g. {1.0, 3.0, 5.0} seconds)
- Multiple speed levels (for speed-trimmable PTGs)

This creates a *state-lattice* planner where edges are kinematically-feasible PTG trajectory segments.

### 3.3 Heuristic Functions

Two heuristics depending on goal type:

- **SE(2) goal** (`default_heuristic_SE2`): Lie group distance (Euclidean + weighted angular distance) plus a heading-alignment term that penalizes the robot for not facing the goal.
- **R(2) goal** (`default_heuristic_R2`): Euclidean distance plus the same heading-alignment penalty.

The heading term `heuristic_heading_weight * |angDistance(atan2(dy,dx), phi)|` encourages the robot to orient towards the goal during transit, which improves path quality for non-holonomic vehicles.

**Weighted A* (`heuristic_epsilon`, default 1.0)**: the OPEN set is ordered by
`f = g + eps * h` (ARA*/SBPL-style); `eps > 1` returns a solution within a
factor `eps` of optimal while expanding fewer nodes. The raw (eps=1)
`costToGoal` is still used for best-node tracking. Measured on BARN (worlds
0-59): `eps` in `[1.5, 2]` cuts median plan time ~4-5x at the cost of 3-8%
longer paths, but does not reduce the worst-case tail (which is
feasibility/resolution-bound, not heuristic-bound). Default stays 1.0 to
preserve resolution-optimality.

### 3.4 Cost Model

Edge cost = `estimatedExecTime` + Σ(cost_evaluators).

- **Base cost**: Estimated execution time of the PTG segment (time = steps × dt).
- **CostEvaluatorCostMap**: Penalizes proximity to obstacles. Uses a 2D grid where each cell stores a cost based on distance to nearest obstacle: `maxCost * (-0.99999 + 1/(d/D))^0.4`, creating a repulsive potential field within clearance distance `D`.
- **CostEvaluatorPreferredWaypoint**: Negative cost (reward) for passing near user-defined waypoints, attracting paths toward desired intermediate locations.

### 3.5 Collision Checking

`tp_obstacles_single_path(k, localObstacles, ptg)` computes the free distance along PTG trajectory `k` given local obstacle points. It uses the PTG's `updateTPObstacleSingle()` which internally uses precomputed collision grids for efficient lookup.

Obstacles are first transformed to the robot's local frame and square-clipped to the PTG's reference distance (`transform_pc_square_clipping`).

### 3.6 Navigation Engine

`NavEngine` is a high-level state machine for waypoint-following navigation:
- Manages a planner thread for async path computation
- Supports enqueued motion commands with odometry-based triggers
- Handles waypoint sequences with skip permissions and look-ahead
- Interfaces with vehicle control via `VehicleMotionInterface`

## 4. Algorithm Flow (TPS_Astar::plan)

```
1. Initialize: create root node at start pose, push to open set
2. Create goal node in lattice
3. While open set not empty and not timed out:
   a. Pop node with lowest f-score
   b. If node is in goal cell → PATH FOUND, break
   c. Mark as visited
   d. find_feasible_paths_to_neighbors():
      - Transform global obstacles to local frame (square clipping)
      - For each PTG:
        - Update dynamic state (current velocity, relative goal)
        - Sample trajectory indices (uniform + direct-to-goal)
        - For each (trajectory, timestamp, speed) triple:
          - Check world bounds
          - Compute free distance via tp_obstacles_single_path()
          - If collision-free, record as candidate neighbor
        - Keep best (shortest distance) path to each lattice cell
   e. For each feasible neighbor:
      - Compute edge cost (exec time + cost evaluators)
      - If tentative g-score < neighbor's g-score:
        - Update neighbor's scores and parent
        - Add/update in open set
        - Insert or rewire edge in motion tree
   f. Optionally save debug visualization
4. Extract result: success flag, path cost, motion tree
```

## 5. Key Design Decisions

- **Exact poses in nodes**: Although nodes are indexed by lattice cells, they store exact SE(2) poses. This avoids accumulating discretization error along the path.
- **PTG dynamic state**: Each PTG evaluation updates the PTG's dynamic state (current velocity, relative goal), enabling velocity-dependent trajectory generation.
- **Multi-PTG support**: Different PTG types can be used simultaneously, and the planner selects the best one for each edge.
- **Rewiring**: When a better path to an existing node is found, the tree is rewired (similar to RRT*), maintaining optimality within the explored lattice.
- **Goal handling**: R(2) goals (position-only) use `sameLocation()` which ignores heading, while SE(2) goals require matching all three coordinates.

---

## 6. Identified Issues and Improvement Plan

### 6.1 Algorithmic / Theoretical Issues

#### ~~P3. `cached_local_obstacles()` Has No Actual Cache~~ ✅ DONE
**Severity**: Medium — performance.

**Implemented**: xy-cell cache keyed by `(ix, iy)` grid indices (heading-independent). Nodes in the same lattice cell share one transformed obstacle cloud; cache is cleared at the start of each `plan()` call. Measured ~6× per-call speedup on a ~960-point cloud. See `tests/test_obstacle_cache.cpp`.

#### P5. Uniform Trajectory Sampling Misses Important Directions
**Severity**: Medium — suboptimal path quality.

Trajectory indices are sampled uniformly (`i * (pathCount-1) / (N-1)`), plus the direct-to-goal direction. This misses trajectories that might navigate around obstacles effectively. In cluttered environments, the planner may fail to find paths that require specific maneuvers between obstacles.

**Fix**: Consider adaptive sampling strategies: (a) bias sampling toward the goal direction with some spread, (b) add obstacle-aware sampling that picks trajectories tangent to nearby obstacles, (c) use the PTG's inverse map to find trajectories passing through promising intermediate points.

#### P6. No Path Smoothing / Post-Processing
**Severity**: Low-Medium — path quality.

The A* output is a sequence of PTG segments snapped to lattice cells. The path can have unnecessary zig-zag or sharp transitions between PTG types. There is a `refine_trajectory` utility, but it only re-parameterizes PTG params for exact poses — it doesn't optimize the path globally.

**Fix**: Add a post-processing step: (a) shortcutting — try to connect non-adjacent nodes directly with PTG segments, (b) elastic band / trajectory optimization to smooth the path while maintaining kinematic feasibility and collision-free status.

#### ~~P7. Edge Cost Uses `estimatedExecTime` But Heuristic Uses Distance~~ ✅ DONE
**Severity**: Medium — inconsistent cost model.

**Implemented**: Heuristic now divides by `maxLinSpeed_` (cached from PTGs at the start of each `plan()` call), returning time units consistent with `estimatedExecTime`. Defaults to 1.0 m/s when called outside `plan()`.

#### ~~P8. No Reverse Motion Support~~  WRONG (reverse is implemented by C-PTG with K=-1)

### 6.2 Implementation Issues

#### ~~I1. `ptgTrimmableSpeed` Not Stored in `path_to_neighbor_t`~~ ✅ DONE
**Severity**: Low — speed is always written as default 1.0 for non-goal paths.

**Implemented**: `path.ptgTrimmableSpeed = tpsPt.speed` is now assigned in the best-path replacement block.

#### I2. `edge_interpolated_path` Off-by-One in Interpolation
**Severity**: Low — minor path quality issue.

Line 62: `const auto iStep = ((i + 1) * ptg_step) / (nSeg + 2);` — divides by `(nSeg + 2)` which means for `nSeg=5`, the intermediate points are at steps 1/7, 2/7, 3/7, 4/7, 5/7 of the total. Combined with the explicit start (step 0) and end (step ptg_step), the total is 7 points. This is correct but the naming `pathInterpolatedSegments=5` is misleading since the actual number of segments in the interpolated path is `nSeg + 1 = 6`, not 5.

**Fix**: Clarify naming, or adjust the formula to produce exactly `nSeg` intermediate points.

#### ~~I3. `transform_pc_square_clipping` Copies by Value~~ ✅ DONE
**Severity**: Low — performance.

**Implemented**: Changed to `const auto&` to bind a reference instead of copying.

#### I4. Incomplete `edge_interpolated_path` Fallback Paths
**Severity**: Low — dead code.

Lines 34-48 have two `THROW_EXCEPTION("To do")` paths for cases where `reconstrRelPoseOpt` or `ptg_stepOpt` are not provided. These throw at runtime if ever hit.

**Fix**: Either implement these paths or remove the optional parameters and require them to always be provided.

#### ~~I5. `NodeCoordsHash` Quality~~ ✅ DONE
**Severity**: Low — potential hash collision performance issue.

**Implemented**: Replaced the `res * 31 + h` rolling hash with the `boost::hash_combine` avalanche pattern (`seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2)`), which properly mixes bits and avoids bucket clustering for adjacent grid coordinates.

### 6.3 Missing Tests

#### ~~T1. No Unit Tests at All~~ ✅ DONE
**Severity**: High — no regression protection.

**Implemented**: GTest suite added under `tests/`. Current coverage:
- `test_costmap.cpp` — CostEvaluatorCostMap correctness (5 tests)
- `test_heuristic.cpp` — heuristic admissibility, consistency, unit correctness (8 tests)
- `test_astar_holonomic.cpp` — end-to-end planning scenarios: free space, diagonal, R(2) goal, SE(2) goal, obstacle avoidance (10 tests)
- `test_edge_interpolation.cpp` — interpolated path start/end/intermediate-point correctness (1 test)
- `test_obstacle_cache.cpp` — cache correctness and performance (3 tests)

### 6.4 Missing Features / TODOs in Code

#### F1. Dynamic Obstacles (line 123 of TPS_Astar.cpp)
`MRPT_TODO("dynamic over future time?")` — currently obstacles are treated as static during planning.

#### F2. Goal Speed (line 185)
`MRPT_TODO("Actually check user input on desired speed at goal")` — goal speed is hardcoded to 0.

#### F3. Speed Zone Filter (line 596)
`MRPT_TODO("Speed zone filter here too?")` — no speed zone filtering is implemented.

#### F4. Non-Zero Final Goal Speed (line 601)
`MRPT_TODO("Support case of final goal speed!=0 ?")` — all goals assume stop at destination.

---

## 7. Prioritized Implementation Plan

### Phase 1: Critical Correctness Fixes ✅ COMPLETE
1. ~~**Fix P7**: Make heuristic units consistent with cost model (time vs distance)~~ ✅
2. ~~**Fix I1**: Store `ptgTrimmableSpeed` in best-path selection~~ ✅

### Phase 2: Unit Test Infrastructure ✅ COMPLETE
3. ~~**T1**: Set up test framework (Google Test via CMake)~~ ✅
4. ~~Write tests for heuristic, cost map, planning scenarios, edge interpolation, obstacle cache~~ ✅

### Phase 3: Performance ✅ COMPLETE
5. ~~**Fix P3**: Implement obstacle cache (spatial index)~~ ✅
6. ~~**Fix I3**: Fix copy-by-value in `transform_pc_square_clipping`~~ ✅
7. ~~**Fix I5**: Better `NodeCoordsHash` (boost::hash_combine avalanche pattern)~~ ✅

### Phase 4: Path Quality
8. **Fix P5**: Adaptive trajectory sampling
9. **Fix P6**: Add path post-processing (shortcutting)

### Phase 5: Features
10. **Fix P8**: Reverse motion support
11. **F1-F4**: Address in-code TODOs (dynamic obstacles, goal speed, speed zones)
