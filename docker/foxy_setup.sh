#!/bin/bash

command0="ros2 launch robot_bringup rs_t265.launch.yaml"

# first source ros2 workspace
source install/setup.bash

# Attach each command in byobu window
byobu new-session -d -s foxy -n window0 "$command0"
byobu attach-session -t foxy