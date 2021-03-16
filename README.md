[![CircleCI](https://circleci.com/gh/jlblancoc/selfdriving.svg?style=svg)](https://circleci.com/gh/jlblancoc/selfdriving)
[![Documentation Status](https://readthedocs.org/projects/selfdriving/badge/?version=latest)](https://selfdriving.readthedocs.io/en/latest/?badge=latest)

# selfdriving
Self-driving (autonomous navigation) algorithms for 2D robots/vehicles based on mrpt-nav

## Build requisites

- MRPT (>=2.0.4)

In Ubuntu, they can installed with:

```
# MRPT >=2.0.4, for now from this PPA (or build from sources if preferred):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev

```


## Demo runs

```
bin/plan-path -g "[5.1 2.0 0]" -s "[-0.2 -0 0]" -p ../share/ptgs_holonomic_robot.ini -o ../share/obstacles_01.txt --min-step-length  0.
```
