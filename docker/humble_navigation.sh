#!/bin/bash

command5="ros2 launch robot_bringup slam.launch.yaml"
command6="ros2 launch robot_bringup navigation.launch.yaml"

# first source ros2 workspace
source install/setup.bash

# Attach each command in byobu window
byobu new-window -t humble:5 -n window5 "$command5"
byobu new-window -t humble:6 -n window6 "$command6"
byobu attach-session -t humble