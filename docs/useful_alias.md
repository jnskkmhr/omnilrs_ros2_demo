# Useful aliases
In field test, you do not want to type long launch command (especially cold winter!). \
Registering alias to launch actuator, camera, LiDAR saves a lot of your effort. 

Below, I listed aliases registered to NUC13 robot PC. \
Just write these in `~/.bash_aliases`.
```bash
alias humble="source /opt/ros/humble/setup.bash"
alias mros-agent-humble="docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6"
alias ros2_ws_setup="source ~/ros2_ws/install/setup.bash"
alias start_d455="ros2 launch robot_bringup rs_launch.launch.py usb_port_id:=4-3.1 camera_name:=d455 camera_namespace:=moonraker depth_emitter:=0 depth_gain:=40 enable_gyro:=true enable_accel:=true"
alias start_d435i="ros2 launch robot_bringup rs_launch.launch.py usb_port_id:=4-3.2 camera_name:=d435i camera_namespace:=moonraker depth_emitter:=0 enable_gyro:=true enable_accel:=true"
alias start_d455_with_color="ros2 launch robot_bringup rs_launch.launch.py usb_port_id:=4-3.1 camera_name:=d455 camera_namespace:=moonraker depth_emitter:=0 enable_gyro:=true enable_accel:=true enable_color:=true"
alias start_d435i_with_color="ros2 launch robot_bringup rs_launch.launch.py usb_port_id:=4-3.2 camera_name:=d435i camera_namespace:=moonraker depth_emitter:=0 enable_gyro:=true enable_accel:=true enable_color:=true"
alias d455_setting="ros2 param set /d455/d455 depth_module.emitter_enabled 0"
alias d455_gain="ros2 param set /d455/d455 depth_module.gain 40"
alias d435i_setting="ros2 param set /d435i/d435i depth_module.emitter_enabled 0"
alias d435i_fix_shutter="ros2 param set /d435i/d435i depth_module.exposure $1"
alias d455_fix_shutter="ros2 param set /d455/d455 depth_module.exposure $1"
alias start_theta="ros2 launch robot_bringup theta_launch.launch.py"
alias start_foxglove_bridge="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 send_buffer_limit:=500000000"
alias start_bag_record="ros2 launch robot_bringup record_bag.launch.py"
```