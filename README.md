[![CI Linux](https://github.com/jlblancoc/selfdriving/actions/workflows/build-linux.yml/badge.svg)](https://github.com/jlblancoc/selfdriving/actions/workflows/build-linux.yml) [![Documentation Status](https://readthedocs.org/projects/selfdriving/badge/?version=latest)](https://selfdriving.readthedocs.io/en/latest/?badge=latest)

# mrpt_path_planning

Path planning and navigation algorithms for robots/vehicles moving on planar environments.
This library builds upon mrpt-nav and the theory behind PTGs to generate libraries of "motion primitives"
for vehicles with arbitrary shape and realistic kinematics and dynamics.

The planner optimizes **SE(2) path cost** (position + heading), not R(2) path length.
For vehicles that rotate, arriving at a goal with the correct heading is part of the
optimal solution — paths that are longer in Euclidean distance but better-aligned
may genuinely have lower cost. See `TPS_Astar.h` for details on the cost model.

## Status on ROS build farm


| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_path_planning__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_path_planning__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_path_planning__ubuntu_jammy_amd64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mrpt_path_planning__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mrpt_path_planning__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) | 
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mrpt_path_planning__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mrpt_path_planning__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) | 
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_path_planning__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_path_planning__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_path_planning__ubuntu_noble_amd64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |

| EOL Distro | Last version |
| ---    | ---    |
| ROS 1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mrpt_path_planning)](https://index.ros.org/?search_packages=true&pkgs=mrpt_path_planning) |


## Build requisites

- [MRPT](https://github.com/MRPT/mrpt/) (>=2.12.0)
- [mvsim](https://github.com/MRPT/mvsim/) (optional to run the live control simulator).

In Ubuntu 22.04 or newer, installed it with:

```
sudo apt install libmrpt-dev
```

For older versions of Ubuntu: 

```
# MRPT from this PPA (or build from sources if preferred, or from ROS package `mrpt2`):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev
```

## Use in your code

From your CMake script:

```
find_package(mrpt_path_planning REQUIRED)
target_link_libraries(YOUR_TARGET mpp::mrpt_path_planning)
```

## Demo runs

### path-planner-cli

Dump default planner parameters to a YAML file for inspection or customization:

```bash
build/bin/path-planner-cli --write-planner-parameters my-planner-params.yaml
```

Plan a path for a **holonomic robot** with an SE(2) goal pose (x y heading_deg),
using a pre-built obstacle point cloud and an obstacle-proximity cost map:

```bash
build/bin/path-planner-cli \
  -s "[0.5 0 0]" \
  -g "[4 2.5 45]" \
  -c share/ptgs_holonomic_robot.ini \
  --obstacles share/obstacles_01.txt \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --costmap-obstacles share/costmap-obstacles.yaml
```

Plan a path with an **R(2) goal** (position only, heading-agnostic), printing the
edge details of the found path and saving the interpolated trajectory to a CSV:

```bash
build/bin/path-planner-cli \
  -s "[0.5 0 0]" \
  -g "[4 2.5]" \
  -c share/ptgs_holonomic_robot.ini \
  --obstacles share/obstacles_01.txt \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --print-path-edges \
  --save-interpolated-path path.csv
```

Plan a path for an **Ackermann (car-like) vehicle**, show the full explored search
tree, and animate the result. Note the goal is given as a position `[x y]` (R²,
heading-agnostic): with arc-based PTGs, arriving at a precise heading AND position
simultaneously is very constrained, so position-only goals are the natural choice
for non-holonomic vehicles:

```bash
build/bin/path-planner-cli \
  -s "[0.5 0 0]" \
  -g "[4 2.5]" \
  -c share/ptgs_ackermann_vehicle.ini \
  --obstacles share/obstacles_01.txt \
  --planner-parameters share/mvsim-demo-astar-planner-params-ackermann.yaml \
  --show-tree \
  --play-animation
```

Plan from an **occupancy grid image** (each pixel = `--obstacles-gridimage-resolution` meters):

```bash
build/bin/path-planner-cli \
  -s "[1.0 1.0 0]" \
  -g "[8.0 6.0 90]" \
  -c share/ptgs_holonomic_robot.ini \
  --obstacles share/map01.png \
  --obstacles-gridimage-resolution 0.05 \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml
```

Plan with a **preferred-waypoints cost layer** to attract the path through
intermediate via-points, plus a proximity cost map:

```bash
build/bin/path-planner-cli \
  -s "[0.5 0 0]" \
  -g "[4 2.5 45]" \
  -c share/ptgs_holonomic_robot.ini \
  --obstacles share/obstacles_01.txt \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --costmap-obstacles share/costmap-obstacles.yaml \
  --waypoints share/mvsim-demo-waypoints01.yaml \
  --waypoints-parameters share/costmap-prefer-waypoints.yaml
```

Enable **verbose debug output** and skip the post-plan refinement stage:

```bash
build/bin/path-planner-cli \
  -s "[0.5 0 0]" \
  -g "[4 2.5 45]" \
  -c share/ptgs_holonomic_robot.ini \
  --obstacles share/obstacles_01.txt \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --no-refine \
  -v DEBUG
```

### selfdriving-simulator-gui (requires mvsim)

GUI with live navigation simulator and A* replanning:

```bash
# Holonomic robot:
build/bin/selfdriving-simulator-gui \
  --waypoints share/mvsim-demo-waypoints01.yaml \
  -s share/mvsim-demo.xml \
  -p share/ptgs_holonomic_robot.ini \
  --nav-engine-parameters share/nav-engine-params.yaml \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --prefer-waypoints-parameters share/costmap-prefer-waypoints.yaml \
  --global-costmap-parameters share/costmap-obstacles.yaml \
  --local-costmap-parameters share/costmap-obstacles.yaml

# Ackermann vehicle:
build/bin/selfdriving-simulator-gui \
  --waypoints share/mvsim-demo-waypoints01.yaml \
  -s share/mvsim-demo.xml \
  -p share/ptgs_ackermann_vehicle.ini \
  --nav-engine-parameters share/nav-engine-params.yaml \
  --planner-parameters share/mvsim-demo-astar-planner-params.yaml \
  --prefer-waypoints-parameters share/costmap-prefer-waypoints.yaml \
  --global-costmap-parameters share/costmap-obstacles.yaml \
  --local-costmap-parameters share/costmap-obstacles.yaml
```
