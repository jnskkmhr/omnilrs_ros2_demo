# JoyStick Controller Package

The package consists of two node: 
1. joy_handler
2. wheel_drive

## Node information
### Node1: joy_handler
* subscriber: subscribes to `joy`(sensor_msgs/Joy) topic from PS4 joystick
* publisher: publishes `cmd_vel`(geometry_msgs/Twist) topic

### Node2: wheel_driver
* subscriber: subscribes to `cmd_vel` topic
* publishes: `throttle` (std_msgs/Float32MultiArray) topic


## Command
```
ros2 launch moonraker_controller teleop_joy.launch.py
```