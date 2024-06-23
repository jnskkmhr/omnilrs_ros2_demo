# Moonraker software stack for simulation

Moonraker project with ROS2 Humble \
Contributor: Junnosuke Kamohara

## Git maintenance rule
- Always put the stable code in `main` branch
- If you want to add any features, create `feature` branch and merge it to `main` later
- Send pull request before merge to main branch (I added branch rule, so you cannot directly merge with main branch.)
- Only upload source code and configuration files (no heavy data!)

## Environment
The repository depends on
- Host computer running simulation (we use IsaacSim)
- Host computer running software stack in this repository

## Setup package
In this project, we use docker for easy deployment.  
You need internet in this setup.

First, clone this repository under your home directory. 
```bash
git clone git@github.com:Space-Robotics-Laboratory/rover_moonraker.git -b sim
```

### Build docker images and ros2 packages
Then, build docker images and ros2 package

```bash
cd rover_moonraker
./docker/build_image.sh
```

```bash
cd rover_moonraker
./docker/run_container.sh

# then build package
colcon build --symlink-install
```


## Run

```bash
./docker/run_container.sh
```
Then, inside container shell, run
```bash
# one-liner script to run slam and navigation 
/docker/humble_navigation.sh
```

## System diagram
Documentation WIP

## Usage
Documentation WIP