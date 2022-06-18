[![Documentation Status](https://readthedocs.org/projects/selfdriving/badge/?version=latest)](https://selfdriving.readthedocs.io/en/latest/?badge=latest)

# selfdriving
Self-driving (autonomous navigation) RRT* (RRT-star) algorithm for 2D robots/vehicles based on mrpt-nav, for vehicles with arbitrary shape and realistic kinematics and dynamics.

## Build requisites

- [MRPT](https://github.com/MRPT/mrpt/) (>=2.3.2)
- [mvsim](https://github.com/MRPT/mvsim/) (optional to run the live control simulator).

In Ubuntu, they can installed with:

```
# MRPT >=2.3.2, for now from this PPA (or build from sources if preferred):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev
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
