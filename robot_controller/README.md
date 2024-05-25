# Robot Controller

The package consists of two nodes: 
1. joy_handler: convert joystick signal to twist
2. wheel_drive: twist to throttle command for left/right motors

## Dependencies
```bash
sudo apt-get install ros-$ROS_DISTRO-joy-linux
```

## Node information
### Node1: joy_handler
* subscriber: subscribes to `joy`(sensor_msgs/Joy) topic from PS4 joystick
* publisher: publishes `cmd_vel`(geometry_msgs/Twist) topic

### Node2: wheel_driver
* subscriber: subscribes to `cmd_vel` topic
* publishes: `throttle` (std_msgs/Float32MultiArray) topic


## Command
```
ros2 launch robot_controller teleop_joy.launch.py
```