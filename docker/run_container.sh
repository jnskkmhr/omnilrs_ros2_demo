#!/bin/bash
xhost +
TASK=$1
IMAGE="moonraker-${TASK}:v1.0"
NAME="moonraker-${TASK}-container"
docker run -it --rm --privileged \
-e "ACCEPT_EULA=Y" \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $HOME/ros2_ws:/ros2_ws \
--name $NAME \
--net host \
$IMAGE