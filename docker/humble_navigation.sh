#!/bin/bash

command0="ros2 launch robot_bringup slam.launch.yaml"
command1="ros2 launch robot_bringup navigation.launch.yaml"

# first source ros2 workspace
source install/setup.bash

# Attach each command in byobu window
byobu new-session -d -s humble_nav -n window0 "$command0"
byobu new-window -t humble_nav:1 -n window1 "$command1"
byobu attach-session -t humble_nav
