# mrpt_path_planning

[![CI Linux](https://github.com/MRPT/mrpt_path_planning/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mrpt_path_planning/actions/workflows/build-linux.yml)
[![License: BSD-3](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/ros/v/jazzy/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#jazzy)

**Kinematically-feasible path planning for robots and vehicles on planar
environments**, for arbitrary robot shapes and realistic kinematics
(differential-drive, Ackermann, holonomic). Built on
[MRPT](https://github.com/MRPT/mrpt/) `mrpt_nav` and the theory of
*Parameterized Trajectory Generators* (PTGs), which act as libraries of motion
primitives.

<p align="center">
  <img src="docs/images/demo-holonomic.svg" width="49%" alt="Holonomic robot following a planned path">
  <img src="docs/images/demo-ackermann.svg" width="49%" alt="Ackermann vehicle following a planned path">
  <br>
  <em>Planned paths for a holonomic robot (left) and an Ackermann vehicle (right).</em>
</p>

<!-- Images generated from mrpt_path_planning_apps/share/ with:
path-planner-cli -s "[0.5 0 0]" -g "[4.2 0.5 80]" \
  -c ptgs_ackermann_vehicle.ini --obstacles obstacles_01.txt \
  --planner-parameters mvsim-demo-astar-planner-params-ackermann.yaml \
  --save-svg demo-ackermann.svg --svg-animate --svg-no-tree --svg-no-bbox \
  --svg-no-status --svg-width 440 --no-gui
(and the same with ptgs_holonomic_robot.ini and
mvsim-demo-astar-planner-params.yaml for demo-holonomic.svg) -->

## Features

- **Kinematically feasible by construction**: every path is a sequence of PTG
  trajectories the vehicle can actually execute, respecting its turning and
  speed limits. Several PTG families can be mixed in one search, and
  speed-trimmable PTGs make the search velocity-aware.
- **Any-shape vehicles**: circular or arbitrary polygonal footprints, including
  concave ones. The footprint is baked once into per-PTG collision grids, so
  the per-node collision cost does not grow with the footprint complexity: one
  pass over the local obstacles gives the free distance along every candidate
  trajectory.
- **No costmap inflation**: the actual footprint is checked against the raw
  obstacle points, with no inflation radius to tune.
- **Deterministic SE(2) lattice A\*** (`mpp::TPS_Astar`) and a
  **bidirectional** variant (`mpp::TPS_Astar_Bidir`). They optimize SE(2) cost
  (position + heading), not just Euclidean length, with optional
  bounded-suboptimal weighted A\* for faster queries.
- **Pose or position goals**: SE(2) goals `[x y phi]` or heading-agnostic R(2)
  goals `[x y]`.
- **Forward and reverse maneuvers**, with an optional **Reeds-Shepp** heuristic
  and analytic goal expansion for tight maneuvers such as parking.
- **Certified collision checking**: the collision grids are conservative, so a
  path reported as free is free for the continuous swept motion.
- **Pluggable cost layers**: obstacle-proximity cost maps and
  preferred-waypoint attractors.
- **Navigation building blocks**: `NavEngine` (waypoint-sequence navigation
  with replanning) and `TrajectoryFollower` (pure pursuit with predictive
  safety).
- **Headless core**: the algorithms library has no GUI dependency.

## Quick start

Install the binary packages (ROS 2 Humble or newer):

```bash
sudo apt install ros-$ROS_DISTRO-mrpt-path-planning
```

Plan a path around some obstacles for a holonomic robot, using the example
configuration files installed with the apps package:

```bash
cd $(ros2 pkg prefix mrpt_path_planning_apps)/share/mrpt_path_planning_apps

path-planner-cli \
  -s "[0.5 0 0]" -g "[4.2 0.5 80]" \
  -c ptgs_holonomic_robot.ini \
  --obstacles obstacles_01.txt \
  --planner-parameters mvsim-demo-astar-planner-params.yaml \
  --play-animation
```

## Packages

| Package | Contents | Depend on it when... |
| --- | --- | --- |
| `mrpt_path_planning_core` | C++ library (namespace `mpp`): PTGs, planners, cost evaluators, `NavEngine`, `TrajectoryFollower`. Depends only on `mrpt_nav`, `mrpt_maps`, `mrpt_graphs`, `mrpt_containers`. | You only need the algorithms (most users). |
| `mrpt_path_planning_apps` | `path-planner-cli`, `selfdriving-simulator-gui`, example config files. Adds `mrpt_gui`, `cli11`, `mvsim`. | You want the command-line and GUI tools. |
| `mrpt_path_planning` | Metapackage depending on the two above. | Backward compatibility with existing consumers. |

ROS 2 integration (planner and trajectory follower nodes) lives in
[mrpt_navigation](https://github.com/mrpt-ros-pkg/mrpt_navigation).

## Using the library

In `package.xml`:

```xml
<depend>mrpt_path_planning_core</depend>
```

In `CMakeLists.txt` (the CMake package is named `mrpt_path_planning`):

```cmake
find_package(mrpt_path_planning REQUIRED)
target_link_libraries(YOUR_TARGET mpp::mrpt_path_planning)
```

Minimal example:

```cpp
#include <mpp/algos/TPS_Astar.h>
#include <mpp/algos/trajectories.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CSimplePointsMap.h>

mpp::PlannerInput in;

// Vehicle kinematics and shape, as a set of PTGs:
mrpt::config::CConfigFile cfg("ptgs_holonomic_robot.ini");
in.ptgs.initFromConfigFile(cfg, "SelfDriving");

// Start pose and goal (a TPose2D goal; use a TPoint2D for position-only):
in.stateStart.pose = {0.0, 0.0, 0.0};
in.stateGoal.state = mrpt::math::TPose2D{4.0, 2.5, 0.5 * M_PI};
in.worldBboxMin    = {-1.0, -1.0, -M_PI};
in.worldBboxMax    = {6.0, 4.0, M_PI};

// Obstacles, as a point cloud:
auto obs = mrpt::maps::CSimplePointsMap::Create();
obs->insertPoint(2.0, 1.0, 0.0);
in.obstacles.push_back(mpp::ObstacleSource::FromStaticPointcloud(obs));

mpp::TPS_Astar planner;
planner.params_.maximumComputationTime = 5.0;  // [s]

const mpp::PlannerOutput out = planner.plan(in);
if (out.success)
{
    // Sequence of motion primitives, and the time-sampled trajectory
    // (relative to the start pose):
    const auto [nodes, edges] = out.motionTree.backtrack_path(*out.goalNodeId);
    const mpp::trajectory_t traj = mpp::plan_to_trajectory(edges, in.ptgs);
}
```

Planner parameters can also be loaded from YAML with
`planner.params_from_yaml()`; run
`path-planner-cli --write-planner-parameters params.yaml` to get a file with
all parameters and their defaults.

## Configuration files

Example files in [`mrpt_path_planning_apps/share/`](mrpt_path_planning_apps/share/):

| File | Purpose |
| --- | --- |
| `ptgs_holonomic_robot.ini`, `ptgs_ackermann_vehicle.ini` | Vehicle model: PTG families, velocity limits, robot shape. |
| `mvsim-demo-astar-planner-params*.yaml` | `TPS_Astar` parameters (lattice resolution, sampling, heuristics). |
| `costmap-obstacles.yaml` | Obstacle-proximity cost map parameters. |
| `costmap-prefer-waypoints.yaml` | Preferred-waypoints cost layer parameters. |
| `mvsim-demo-waypoints*.yaml` | Example waypoint sequences. |
| `nav-engine-params.yaml` | `NavEngine` parameters. |
| `obstacles_01.txt`, `map0*.png` | Example obstacles: point list or occupancy grid image. |
| `mvsim-demo.xml` | [mvsim](https://github.com/MRPT/mvsim/) world for the simulator demo. |

## Demos

### path-planner-cli

All the following commands are run from the directory with the example files
(`mrpt_path_planning_apps/share/` in the sources, or
`$(ros2 pkg prefix mrpt_path_planning_apps)/share/mrpt_path_planning_apps` once
installed), and start from this base command (holonomic robot, SE(2) goal `[x y heading_deg]`):

```bash
path-planner-cli \
  -s "[0.5 0 0]" -g "[4.2 0.5 80]" \
  -c ptgs_holonomic_robot.ini \
  --obstacles obstacles_01.txt \
  --planner-parameters mvsim-demo-astar-planner-params.yaml
```

| Scenario | Add or change |
| --- | --- |
| Obstacle-proximity cost map | `--costmap-obstacles costmap-obstacles.yaml` |
| R(2) goal (position only), print path edges, save trajectory to CSV | `-g "[4.2 0.5]" --print-path-edges --save-interpolated-path path.csv` |
| Ackermann vehicle, show search tree and animation | `-c ptgs_ackermann_vehicle.ini --planner-parameters mvsim-demo-astar-planner-params-ackermann.yaml --show-tree --play-animation` |
| Occupancy grid image as obstacles | `-s "[1 1 0]" -g "[8 6 90]" --obstacles map01.png --obstacles-gridimage-resolution 0.05` |
| Attract the path through via-points | `--waypoints mvsim-demo-waypoints01.yaml --waypoints-parameters costmap-prefer-waypoints.yaml` |
| Save a 2D SVG plot, no GUI | `--save-svg plan.svg --no-gui` |
| Save an animated SVG of the robot following the path | `--save-svg plan.svg --svg-animate --no-gui` |
| Cleaner SVG for figures: no search tree, box or label, custom width | `--svg-no-tree --svg-no-bbox --svg-no-status --svg-width 600` |
| Verbose output, skip path refinement | `-v DEBUG --no-refine` |

With forward and reverse circular-arc PTGs (as in `ptgs_ackermann_vehicle.ini`),
pose goals are reached exactly through Reeds-Shepp maneuvers. Position-only
goals end within one lattice cell (`grid_resolution_xy`) of the goal point.
Run `path-planner-cli --help` for all options.

### selfdriving-simulator-gui

Live navigation in the [mvsim](https://github.com/MRPT/mvsim/) simulator, with
`NavEngine` and A\* replanning.

<details>
<summary>Commands</summary>

```bash
# Holonomic robot (use ptgs_ackermann_vehicle.ini for an Ackermann vehicle):
selfdriving-simulator-gui \
  --waypoints mvsim-demo-waypoints01.yaml \
  -s mvsim-demo.xml \
  -p ptgs_holonomic_robot.ini \
  --nav-engine-parameters nav-engine-params.yaml \
  --planner-parameters mvsim-demo-astar-planner-params.yaml \
  --prefer-waypoints-parameters costmap-prefer-waypoints.yaml \
  --global-costmap-parameters costmap-obstacles.yaml \
  --local-costmap-parameters costmap-obstacles.yaml
```

</details>

## Building from source

Requirements:

- [MRPT](https://github.com/MRPT/mrpt/) 3.x (its colcon modules `mrpt_nav`,
  `mrpt_maps`, ...).
- [colcon](https://colcon.readthedocs.io/): this repository is colcon-only;
  there is no standalone top-level CMake build.
- Optional: [mvsim](https://github.com/MRPT/mvsim/), for the live simulator.

Install dependencies either from the ROS 2 repositories:

```bash
sudo apt install ros-$ROS_DISTRO-mrpt-nav ros-$ROS_DISTRO-mrpt-gui python3-colcon-common-extensions
```

or, without ROS, from the MRPT 3 PPA (as in this repository's CI):

```bash
sudo add-apt-repository ppa:joseluisblancoc/mrpt3-stable
sudo apt update
sudo apt install libmrpt-dev libcli11-dev python3-colcon-common-extensions
```

Then build and test from the workspace containing this repository:

```bash
colcon build --base-paths .
colcon test --base-paths . && colcon test-result --verbose
source install/setup.bash
```

## How it works

A PTG maps a whole family of kinematically-feasible trajectories to a compact
*trajectory-parameter space* (TP-Space). Obstacles are projected into that
space using precomputed collision grids, so checking many candidate motions
against a point cloud is cheap. `TPS_Astar` runs A\* over an SE(2) lattice
whose edges are PTG trajectory segments, each node storing its exact
(non-snapped) pose. Edge cost is the estimated execution time plus the
configured cost layers, so a path that is longer in distance but reaches the
goal with the right heading may be optimal. See
[`AGENTS.md`](AGENTS.md) for a more detailed design overview.

## ROS build farm status

<details>
<summary>Build and release status per distro</summary>


| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_path_planning__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_path_planning__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#humble) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mrpt_path_planning__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mrpt_path_planning__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#jazzy) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mrpt_path_planning__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mrpt_path_planning__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/kilted/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#kilted) |
| ROS 2 Lyrical (u26.04) | [![Build Status](https://build.ros2.org/job/Ldev__mrpt_path_planning__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mrpt_path_planning__ubuntu_resolute_amd64/) | [![Version](https://img.shields.io/ros/v/lyrical/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#lyrical) |
| ROS 2 Rolling (u26.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_path_planning__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_path_planning__ubuntu_resolute_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mrpt_path_planning)](https://index.ros.org/?pkgs=mrpt_path_planning&search_packages=true#rolling) |

Binary package build status per package, distro, OS and architecture
(Ubuntu `amd64` and `arm64`, plus RHEL and Fedora `x86_64` where the distro
targets them):

| Package | ROS 2 Humble <br/> BinBuild | ROS 2 Jazzy <br/> BinBuild | ROS 2 Kilted <br/> BinBuild | ROS 2 Lyrical <br/> BinBuild | ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- | --- | --- |
| mrpt_path_planning | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning__ubuntu_jammy_arm64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning__fedora_43_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning__fedora_43_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning__fedora_44_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning__fedora_44_x86_64__binary/) |
| mrpt_path_planning_core | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning_core__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning_core__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning_core__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning_core__ubuntu_jammy_arm64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning_core__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning_core__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning_core__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning_core__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning_core__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning_core__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning_core__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning_core__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning_core__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning_core__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning_core__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning_core__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning_core__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning_core__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning_core__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning_core__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning_core__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning_core__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning_core__fedora_43_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning_core__fedora_43_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning_core__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning_core__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning_core__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning_core__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning_core__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning_core__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning_core__fedora_44_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning_core__fedora_44_x86_64__binary/) |
| mrpt_path_planning_apps | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning_apps__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning_apps__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning_apps__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mrpt_path_planning_apps__ubuntu_jammy_arm64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning_apps__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning_apps__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning_apps__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mrpt_path_planning_apps__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning_apps__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Jbin_rhel_el964__mrpt_path_planning_apps__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning_apps__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning_apps__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning_apps__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mrpt_path_planning_apps__ubuntu_noble_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning_apps__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Kbin_rhel_el964__mrpt_path_planning_apps__rhel_9_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning_apps__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mrpt_path_planning_apps__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning_apps__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__mrpt_path_planning_apps__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning_apps__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_rhel_el1064__mrpt_path_planning_apps__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning_apps__fedora_43_x86_64__binary/badge/icon)](https://build.ros2.org/job/Lbin_fedora_fc4364__mrpt_path_planning_apps__fedora_43_x86_64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning_apps__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mrpt_path_planning_apps__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning_apps__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mrpt_path_planning_apps__ubuntu_resolute_arm64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning_apps__rhel_10_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el1064__mrpt_path_planning_apps__rhel_10_x86_64__binary/) <br> [![Build Status](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning_apps__fedora_44_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_fedora_fc4464__mrpt_path_planning_apps__fedora_44_x86_64__binary/) |

| EOL Distro | Last version |
| ---    | ---    |
| ROS 1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |


</details>

## Publications

This library builds on the Trajectory Parameter Space (TP-Space) line of work:

- J.L. Blanco, J. Gonzalez, J.A. Fernandez-Madrigal, "The Trajectory Parameter
  Space (TP-Space): A New Space Representation for Non-Holonomic Mobile Robot
  Reactive Navigation", *IEEE/RSJ Int. Conf. on Intelligent Robots and Systems
  (IROS)*, pp. 1462-1468, 2006.
  [doi:10.1109/IROS.2006.282518](https://doi.org/10.1109/IROS.2006.282518)
- J.L. Blanco, J. Gonzalez, J.A. Fernandez-Madrigal, "Extending obstacle
  avoidance methods through multiple parameter-space transformations",
  *Autonomous Robots*, 24(1):29-48, 2008.
- J.L. Blanco, M. Bellone, A. Gimenez-Fernandez, "TP-Space RRT: Kinematic
  Path Planning of Non-Holonomic Any-Shape Vehicles", *International Journal
  of Advanced Robotic Systems*, 12(5):55, 2015.
  [doi:10.5772/60463](https://doi.org/10.5772/60463)
- A paper describing the planner in this repository will be available on arXiv
  (2026), soon. <!-- TODO: add arXiv reference and BibTeX -->

## License

BSD 3-Clause. See [LICENSE](LICENSE).
