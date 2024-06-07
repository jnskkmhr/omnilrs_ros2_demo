#!/bin/bash
xhost +
IMAGE="moonraker:v1.0"
NAME="moonraker-container"
docker run -it --rm --privileged \
-e "ACCEPT_EULA=Y" \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $HOME/ros2_ws:/ros2_ws \
--name $NAME \
$IMAGE