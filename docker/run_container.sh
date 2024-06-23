#!/bin/bash
xhost +local:root
IMAGE="moonraker-navigation:v1.0"
NAME="moonraker-navigation-container"
WS="$HOME/junnosuke/rover_moonraker"

ROS_VERSION="humble"
DOCKER_RUN_CMD="docker run -it --rm --privileged \
                -e "ACCEPT_EULA=Y" \
                -e DISPLAY \
                -e "PRIVACY_CONSENT=Y" \
                -v $HOME/.Xauthority:/root/.Xauthority \
                -v /dev/:/dev/
                -v $WS/docker:/docker \
                -v $WS/${ROS_VERSION}_ws:/ros2_ws \
                --net host \
                --ipc=host \
                --name $NAME \
                $IMAGE"

exec ${DOCKER_RUN_CMD[*]}
