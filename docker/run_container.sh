#!/bin/bash
xhost +local:root
IMAGE="omnilrs-navigation:v1.0"
NAME="omnilrs-navigation-container"
DOCKER_RUN_CMD="docker run -it --rm --privileged \
                -e "ACCEPT_EULA=Y" \
                -e DISPLAY \
                -e "PRIVACY_CONSENT=Y" \
                -v $HOME/.Xauthority:/root/.Xauthority \
                -v /dev/:/dev/
                -v $PWD/docker:/docker \
                -v $PWD/humble_ws:/ros2_ws \
                --net host \
                --ipc=host \
                --name $NAME \
                $IMAGE"

exec ${DOCKER_RUN_CMD[*]}