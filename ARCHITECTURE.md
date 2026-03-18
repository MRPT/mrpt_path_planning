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

#### P3. `cached_local_obstacles()` Has No Actual Cache
**Severity**: Medium — performance.

The function has a `MRPT_TODO("Impl actual cache")` and recomputes local obstacles from scratch for every node expansion. For dense obstacle maps, this `O(N_obstacles)` transform-and-clip operation per node is a major bottleneck.

**Fix**: Implement a spatial cache (e.g., grid-indexed or KD-tree-based) that avoids re-transforming the same obstacles when consecutive queries are spatially close. Or use a spatial index on the global obstacles (MRPT's built-in KD-tree) with range queries.

#### P5. Uniform Trajectory Sampling Misses Important Directions
**Severity**: Medium — suboptimal path quality.

Trajectory indices are sampled uniformly (`i * (pathCount-1) / (N-1)`), plus the direct-to-goal direction. This misses trajectories that might navigate around obstacles effectively. In cluttered environments, the planner may fail to find paths that require specific maneuvers between obstacles.

**Fix**: Consider adaptive sampling strategies: (a) bias sampling toward the goal direction with some spread, (b) add obstacle-aware sampling that picks trajectories tangent to nearby obstacles, (c) use the PTG's inverse map to find trajectories passing through promising intermediate points.

#### P6. No Path Smoothing / Post-Processing
**Severity**: Low-Medium — path quality.

The A* output is a sequence of PTG segments snapped to lattice cells. The path can have unnecessary zig-zag or sharp transitions between PTG types. There is a `refine_trajectory` utility, but it only re-parameterizes PTG params for exact poses — it doesn't optimize the path globally.

**Fix**: Add a post-processing step: (a) shortcutting — try to connect non-adjacent nodes directly with PTG segments, (b) elastic band / trajectory optimization to smooth the path while maintaining kinematic feasibility and collision-free status.

#### P7. Edge Cost Uses `estimatedExecTime` But Heuristic Uses Distance
**Severity**: Medium — inconsistent cost model.

`Planner::cost_path_segment()` uses `edge.estimatedExecTime` as base cost (time-optimal objective), but the heuristic (`default_heuristic_SE2/R2`) returns a geometric distance (Lie metric / Euclidean). These are in different units. This inconsistency means the heuristic can over- or under-estimate depending on the robot's speed, compounding the admissibility issue from P1.

**Fix**: Make the heuristic consistent with the cost model. For time-optimal planning, the heuristic should be `distance / max_speed`. For distance-optimal planning, the edge cost should use `ptgDist` instead of `estimatedExecTime`.

#### P8. No Reverse Motion Support
**Severity**: Low — limits applicability.

The planner only explores forward PTG trajectories. For Ackermann or differential-drive robots in tight spaces (parking, U-turns), reverse motion is essential.

**Fix**: Add reverse-motion PTGs or a separate reverse-driving PTG set. The A* framework already supports multiple PTGs, so reverse PTGs can be added as additional entries in the PTG vector.

### 6.2 Implementation Issues

#### I1. `ptgTrimmableSpeed` Not Stored in `path_to_neighbor_t`
**Severity**: Low — speed is always written as default 1.0 for non-goal paths.

When building `bestPaths` in `find_feasible_paths_to_neighbors`, if a shorter path replaces an existing one, `path.ptgTrimmableSpeed` is never updated from `tpsPt.speed` (it keeps its default 1.0). The speed from the TPS point is lost.

Looking more carefully: line 788 updates `path.ptgDist`, `ptgIndex`, `ptgTrajIndex`, etc., but does NOT update `ptgTrimmableSpeed`. So the speed modulation used during collision-free evaluation is lost.

**Fix**: Add `path.ptgTrimmableSpeed = speed;` (where `speed` is the current `tpsPt.speed`) in the `if (relTrgDist < path.ptgDist)` block.

#### I2. `edge_interpolated_path` Off-by-One in Interpolation
**Severity**: Low — minor path quality issue.

Line 62: `const auto iStep = ((i + 1) * ptg_step) / (nSeg + 2);` — divides by `(nSeg + 2)` which means for `nSeg=5`, the intermediate points are at steps 1/7, 2/7, 3/7, 4/7, 5/7 of the total. Combined with the explicit start (step 0) and end (step ptg_step), the total is 7 points. This is correct but the naming `pathInterpolatedSegments=5` is misleading since the actual number of segments in the interpolated path is `nSeg + 1 = 6`, not 5.

**Fix**: Clarify naming, or adjust the formula to produce exactly `nSeg` intermediate points.

#### I3. `transform_pc_square_clipping` Copies by Value
**Severity**: Low — performance.

Line 14: `const auto obs_xs = inMap.getPointsBufferRef_x()` copies the entire X coordinate vector. The `Ref` suffix suggests it should be a reference, but `auto` makes a copy. Same for `obs_ys`.

**Fix**: Use `const auto& obs_xs = ...` to avoid the copy.

#### I4. Incomplete `edge_interpolated_path` Fallback Paths
**Severity**: Low — dead code.

Lines 34-48 have two `THROW_EXCEPTION("To do")` paths for cases where `reconstrRelPoseOpt` or `ptg_stepOpt` are not provided. These throw at runtime if ever hit.

**Fix**: Either implement these paths or remove the optional parameters and require them to always be provided.

#### I5. `NodeCoordsHash` Quality
**Severity**: Low — potential hash collision performance issue.

The hash function `res = res * 31 + hash(field)` is simple and can produce collisions for grid coordinates with small ranges. For a typical planning scenario with ~1000 cells in each dimension, this may not matter, but for large environments the lattice can have millions of cells.

**Fix**: Consider using a better hash combiner (e.g., `boost::hash_combine` pattern or a mixing function like `hash ^ (hash >> 16)`).

### 6.3 Missing Tests

#### T1. No Unit Tests at All
**Severity**: High — no regression protection.

The project has zero unit tests. For a planning library where correctness is safety-critical, this is a significant gap.

**Proposed test suite**:

1. **Lattice discretization tests**: Verify `x2idx`, `y2idx`, `phi2idx` round-trip consistency. Test edge cases at cell boundaries and around ±π for phi.

2. **Heuristic tests**: Verify heuristic returns 0 at goal, positive elsewhere. Test consistency (h(x) ≤ cost(x,y) + h(y) for neighbors). Test symmetry properties.

3. **Collision checking tests**: Create a PTG, place known obstacles, verify `tp_obstacles_single_path` returns correct free distances. Test edge cases: obstacle at origin, obstacle exactly on trajectory, no obstacles.

4. **Simple planning scenarios**:
   - Straight line (no obstacles): verify path found, cost is reasonable.
   - Single obstacle: verify path goes around it.
   - Narrow passage: verify path threads through.
   - Unreachable goal (surrounded by obstacles): verify failure reported.
   - Goal = start: verify trivial solution.

5. **Cost evaluator tests**: Verify CostEvaluatorCostMap produces expected costs for known obstacle/pose configurations.

6. **Edge interpolation tests**: Verify interpolated path starts at origin, ends at target pose, intermediate points are on the PTG trajectory.

7. **Tree operations tests**: insert_node_and_edge, rewire_node_parent, backtrack_path correctness.

8. **R(2) vs SE(2) goal tests**: Verify heading-agnostic goals work correctly.

9. **Timeout test**: Verify planner respects `maximumComputationTime`.

10. **Multi-PTG tests**: Verify planner can use different PTG types in the same plan.

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

### Phase 1: Critical Correctness Fixes
1. **Fix P7**: Make heuristic units consistent with cost model (time vs distance)
2. **Fix I1**: Store `ptgTrimmableSpeed` in best-path selection

### Phase 2: Unit Test Infrastructure
4. **T1**: Set up test framework (e.g., Google Test via CMake)
5. Write tests for lattice discretization, collision checking, simple planning scenarios, tree operations

### Phase 3: Performance
6. **Fix P3**: Implement obstacle cache (spatial index)
7. **Fix I3**: Fix copy-by-value in `transform_pc_square_clipping`

### Phase 4: Path Quality
8. **Fix P2**: Clamp and smooth costmap function
9. **Fix P5**: Adaptive trajectory sampling
10. **Fix P6**: Add path post-processing (shortcutting)

### Phase 5: Features
11. **Fix P1**: Proper angle wrapping in bbox checks
12. **Fix P8**: Reverse motion support
13. **F1-F4**: Address in-code TODOs (dynamic obstacles, goal speed, speed zones)
