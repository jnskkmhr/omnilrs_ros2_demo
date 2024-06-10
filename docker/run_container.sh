#!/bin/bash
xhost +
TASK=$1
IMAGE="moonraker-${TASK}:v1.0"
NAME="moonraker-${TASK}-container"

if [${TAG} = "perception"]; then
    ROS_VERSION="foxy"
elif [${TAG} = "navigation"]; then
    ROS_VERSION="humble"
else
    echo "WRONG TAG"
    exit 0

docker run -it --rm --privileged \
-e "ACCEPT_EULA=Y" \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $HOME/rover_moonraker/${ROS_VERSION}_ws:/ros2_ws \
--name $NAME \
$IMAGE