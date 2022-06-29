[![CI Linux](https://github.com/jlblancoc/selfdriving/actions/workflows/build-linux.yml/badge.svg)](https://github.com/jlblancoc/selfdriving/actions/workflows/build-linux.yml) [![Documentation Status](https://readthedocs.org/projects/selfdriving/badge/?version=latest)](https://selfdriving.readthedocs.io/en/latest/?badge=latest)

# selfdriving
Self-driving (autonomous navigation) algorithms for robots/vehicles moving on planar environments. This builds upon mrpt-nav and the theory behind PTGs to generate libraries of "motion primitives" for vehicles with arbitrary shape and realistic kinematics and dynamics.

## Build requisites

- [MRPT](https://github.com/MRPT/mrpt/) (>=2.3.2)
- [mvsim](https://github.com/MRPT/mvsim/) (optional to run the live control simulator).

In Ubuntu 22.04 or newer, installed it with:

```
sudo apt install libmrpt-dev
```

For older versions of Ubuntu: 

```
# MRPT >=2.3.2, for now from this PPA (or build from sources if preferred):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev
```

## Use in your code

From your CMake script:

```
find_package(selfdriving REQUIRED)
target_link_libraries(YOUR_TARGET selfdriving::selfdriving)
```

## Demo runs

```
# bin/path-planner-cli --write-planner-parameters tps-rrtstar.yaml
```

Command-line app to test the A* planner:

```
bin/path-planner-cli -g "[4 2.5 45]" -s "[0.5 0 0]" \
    --planner "selfdriving::TPS_Astar"  \
    -c ../share/ptgs_holonomic_robot.ini  \
    --obstacles ../share/obstacles_01.txt  \
    --planner-parameters ../share/mvsim-demo-astar-planner-params.yaml  \
    --costmap
```

```
bin/path-planner-cli --write-planner-parameters tps-rrtstar.yaml
bin/path-planner-cli -g "[4 2.5 45]" -s "[0.5 0 0]" \
  -p ../share/ptgs_holonomic_robot.ini \
  --obstacles ../share/obstacles_01.txt \
  --planner-parameters tps-rrtstar.yaml \
  --max-iterations 1000 \
  --costmap \
  --random-seed 3
# Edit tps-rrtstar.yaml as desired and re-run
```

GUI with live navigation simulator:

```
bin/selfdriving-simulator-gui \
  --waypoints ../share/mvsim-demo-waypoints01.yaml \
  -s ../share/mvsim-demo.xml \
  -p ../share/ptgs_holonomic_robot.ini \
  --planner-parameters ../share/mvsim-demo-astar-planner-params.yaml \
  -v DEBUG
```
