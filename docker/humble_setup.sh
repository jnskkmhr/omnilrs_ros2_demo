#!/bin/bash

command0="ros2 launch robot_bringup foxglove.launch.yaml"

# first source ros2 workspace
source install/setup.bash

# Attach each command in byobu window
byobu new-session -d -s humble -n window0 "$command0"
byobu attach-session -t humble