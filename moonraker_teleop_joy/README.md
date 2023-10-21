# PS4 joystick operation

## prerequisits
- Install ROS2 foxy
- Install joy related binary
```
sudo apt install ros-foxy-joy-linux
sudo apt install ros-foxy-joy-linux-dbgsym
```

## Build this package
```
cd {/path/to/ros2_ws}
colcon build --symlink-install
```

## Launch
```
ros2 launch moonraker_teleop_joy moonraker_teleop_joy.launch.py
```