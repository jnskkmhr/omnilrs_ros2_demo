#!/bin/bash
xhost +local:root
IMAGE="omnilrs-navigation:v1.0"
NAME="omnilrs-navigation-container"
DOCKER_RUN_CMD="docker run -it --rm --privileged \
                --gpus all \
                --runtime=nvidia \
                -e NVIDIA_VISIBLE_DEVICES=all \
                -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e XDG_RUNTIME_DIR=/tmp/runtime-docker \
                -e "ACCEPT_EULA=Y" \
                -e "PRIVACY_CONSENT=Y" \
                -e "DISPLAY=$DISPLAY" \
                -v $HOME/.Xauthority:/root/.Xauthority \
                -v /dev/:/dev/ \
                -v $PWD/docker:/docker \
                -v $PWD/humble_ws:/ros2_ws \
                --network=host \
                --ipc=host \
                --name $NAME \
                $IMAGE"

# Create runtime directory
mkdir -p /tmp/runtime-docker

exec ${DOCKER_RUN_CMD[*]}