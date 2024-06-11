#!/bin/bash
TASK=$1
IMAGE="moonraker-${TASK}:v1.0"
NAME="moonraker-${TASK}-container"

if [ $TASK = "perception" ]; then
    ROS_VERSION="foxy"
    DOCKER_RUN_CMD="docker run -it --rm --privileged \
                    -e "ACCEPT_EULA=Y" \
                    -e DISPLAY \
                    -e "PRIVACY_CONSENT=Y" \
                    -v $HOME/.Xauthority:/root/.Xauthority \
                    -v /dev/:/dev/ \
                    -v $HOME/rover_moonraker/docker:/docker \
                    -v $HOME/rover_moonraker/${ROS_VERSION}_ws:/ros2_ws \
                    --name $NAME \
		    --net host \
                    $IMAGE"
elif [ $TASK = "navigation" ]; then
    ROS_VERSION="humble"
    DOCKER_RUN_CMD="docker run -it --rm --privileged \
                    -e "ACCEPT_EULA=Y" \
                    -e DISPLAY \
                    -e "PRIVACY_CONSENT=Y" \
                    -v $HOME/.Xauthority:/root/.Xauthority \
                    -v /dev/:/dev/ \
                    -v $HOME/rover_moonraker/docker:/docker \
                    -v $HOME/rover_moonraker/${ROS_VERSION}_ws:/ros2_ws \
                    -p 8765:8765 \
                    --name $NAME \
		    --net host \
                    $IMAGE"
elif [ $TASK = "micro-ros" ]; then
    DOCKER_RUN_CMD="docker run -it --rm -v /dev:/dev --privileged \
                    --net=host microros/micro-ros-agent:humble \
                    serial --dev /dev/ttyUSB0 -v6"
else
    echo "Specified task does not exist..."
    exit 0
fi

exec ${DOCKER_RUN_CMD[*]}
