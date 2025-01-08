#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/ros2_ws
RUN rosdep install --from-paths src --ignore-src -r -y
exec "$@"