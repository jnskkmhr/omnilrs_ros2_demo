# ROS2 docker for the OmniLRS demo

ROS2 humble container to run OmniLRS demo.\
In this project, we use docker for easy development.

First, clone this repository under your home directory. 
```bash
git clone git@github.com:jnskkmhr/omnilrs_ros2_demo.git
```

### Build docker images and ros2 packages
Then, build docker images and ros2 package

```bash
cd omnilrs_ros2_demo
./docker/build_image.sh
```

```bash
cd omnilrs_ros2_demo
./docker/run_container.sh

# then build package
colcon build --symlink-install
```


## Run

### joystick teleoperation
Inside container shell, run
```bash
ros2 launch robot_controller teleop_joy.launch.yaml
```
If you want to change the max speed, please change parameters in [joystick.yaml](humble_ws/src/robot_controller/launch/teleop_joy.launch.yaml)

### navigation

Inside container shell, run
```bash
# one-liner script to run slam and navigation 
/docker/humble_navigation.sh
```