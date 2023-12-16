# Moonraker bringup package (WIP)

Enables the following in one launch file

1. Launch Multiple Realsense Camera
2. Launch graph slam (slam_toolbox) + Nav2 server
3. Launch ros2 bag record


## RealSense

Register camera launch command as aliases since it is tedious to type long command all the time.\
Please replace usb_port_id with your own port setting.
```
alias start_d455="ros2 launch moonraker_bringup rs_launch.launch.py usb_port_id:=4-3.1 camera_name:=d455 camera_namespace:=moonraker depth_emitter:=0 depth_gain:=40 enable_gyro:=true enable_accel:=true"
alias start_d435i="ros2 launch moonraker_bringup rs_launch.launch.py usb_port_id:=4-3.2 camera_name:=d435i camera_namespace:=moonraker depth_emitter:=0 enable_gyro:=true enable_accel:=true"
```

Then, call
```
start_d455
start_d435i
```

By default you will see the following topics:
```
/d435i/accel/imu_info
/d435i/accel/metadata
/d435i/accel/sample
/d435i/extrinsics/depth_to_accel
/d435i/extrinsics/depth_to_gyro
/d435i/extrinsics/depth_to_infra1
/d435i/extrinsics/depth_to_infra2
/d435i/gyro/imu_info
/d435i/gyro/metadata
/d435i/gyro/sample
/d435i/imu
/d435i/infra1/camera_info
/d435i/infra1/image_rect_raw
/d435i/infra1/metadata
/d435i/infra2/camera_info
/d435i/infra2/image_rect_raw
/d435i/infra2/metadata
/d455/accel/imu_info
/d455/accel/metadata
/d455/accel/sample
/d455/extrinsics/depth_to_accel
/d455/extrinsics/depth_to_gyro
/d455/extrinsics/depth_to_infra1
/d455/extrinsics/depth_to_infra2
/d455/gyro/imu_info
/d455/gyro/metadata
/d455/gyro/sample
/d455/imu
/d455/infra1/camera_info
/d455/infra1/image_rect_raw
/d455/infra1/metadata
/d455/infra2/camera_info
/d455/infra2/image_rect_raw
/d455/infra2/metadata
/parameter_events
/rosout
/tf_static
```

## Rosbag2
Save ros2 bag as mcap format.
```
ros2 launch moonraker_bringup record_bag.launch.py
```