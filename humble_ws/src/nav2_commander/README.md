# `tb4_nav2_commander`
Navigation node based on `turtlebot4_navigation` package.

## Before you start
Always **source ROS2 workspace before you begin**!
```bash
source ~/ros/ros2_ws/install/setup.bash
# or use alias registered
source_ros2
```

### Commander (w/o amcl localizer)

```bash
# terminal 5 (Host)
ros2 run nav2_commander run_commander --ros-args -p map:=/path/to/map -p localizer:=custom
```

</div></details>