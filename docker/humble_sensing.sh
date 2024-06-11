#!/bin/bash

command0="ros2 launch robot_controller skid_steer.launch.yaml"
command1="ros2 launch robot_bringup foxglove.launch.yaml"
command2="ros2 launch robot_bringup rs_d455.launch.yaml"
command3="ros2 launch robot_bringup scan_generator.launch.yaml"
command4="ros2 launch robot_bringup static_tf.launch.yaml"

# first source ros2 workspace
source install/setup.bash

# Attach each command in byobu window
byobu new-session -d -s humble -n window0 "$command0"
byobu new-window -t humble:1 -n window1 "$command1"
byobu new-window -t humble:2 -n window2 "$command2"
byobu new-window -t humble:3 -n window3 "$command3"
byobu new-window -t humble:4 -n window4 "$command4"
byobu attach-session -t humble