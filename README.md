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

# then build package & source
colcon build --symlink-install
source install/setup.bash
```


## Run
Make sure that you have installed the docker version of [OmniLRS](https://github.com/AntoineRichard/OmniLRS) and run the simulation.
```bash
# Run the simulation docker
./omnilrs.docker/run_docker.sh
# Launch simulation
/isaac-sim/python.sh run.py environment=lunalab
```
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

## Usage for other visualisation tools

### Foxglove Studio
<img src="media/foxglove.png">

* You might need to install foxglove studio into your desktop, following the official website [installation guide](https://docs.foxglove.dev/docs/foxglove-agent/installation)
* Once the OmniLRS is running, you can run the following command to display data information (i.e. `/imu`, `/odom`, `/depth_img`).

```bash
cd omnilrs_ros2_demo
./docker/run_container.sh

source install/setup.bash

ros2 launch vis_tool foxglove_depth_encode.launch.py
```
> [!TIP]
> You can use the example of our configuration by cliking at `LAYOUT` and `Import from file...`. Then navigate to the `/omnilrs_ros2_demo/humble_ws/src/vis_tool/config/OmniLRS_ros2.json`
> For the given example of configuration file, we assume that you are running the OmniLRS with `ros2_husky_PhysC_vlp16_mono_depth_imu.usd`. You can check this by looking into the `OmniLRS/cfg/environment/<ENVIRONMENT.yaml>`
> The navigation feature is not included in this layout.

### Rerun.io
<img src="media/rerun.png">

Once the OmniLRS is running, you can run the following command to display the same data inside rerun.
```bash
cd omnilrs_ros2_demo
./docker/run_container.sh

source install/setup.bash

python3 src/vis_tool/scripts/rerun/rerun_omnilrs.py
```
