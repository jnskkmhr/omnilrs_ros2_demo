#!/bin/bash
xhost +
IMAGE="moonraker:v1.0"
NAME="moonraker-container"
docker run --name $NAME -it --rm \
-e "ACCEPT_EULA=Y" --privileged \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
--net host \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $HOME/ros2_ws:/ros2_ws \ 
$IMAGE