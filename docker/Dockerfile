FROM osrf/ros:humble-desktop
ENV ROS_DISTRO humble

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        git \
        build-essential \
        cmake \
        byobu \
        vim \
        wget \
        unzip \
        libeigen3-dev \
        python3-colcon-common-extensions \
        libopencv-dev \
        ros-$ROS_DISTRO-joy-linux \
        ros-$ROS_DISTRO-image-geometry && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Install RealSense ROS2 (Including librealsense2)
RUN mkdir -p /etc/apt/keyrings && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN apt-get update && apt-get install -y apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
RUN apt-get install -y ros-humble-realsense2-*

# Install image compression package
RUN apt-get install -y \
ros-${ROS_DISTRO}-image-transport \
ros-${ROS_DISTRO}-image-transport-plugins

# Install nav2 and slamtoolbox
RUN sudo apt install -y \
ros-${ROS_DISTRO}-navigation2 \
ros-${ROS_DISTRO}-nav2-bringup \
ros-${ROS_DISTRO}-slam-toolbox

# Build packages from source
WORKDIR /thirdparty/ros2_ws/src
RUN git clone https://github.com/ros-perception/depthimage_to_laserscan.git -b ros2
WORKDIR /thirdparty/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --symlink-install --packages-select \
depthimage_to_laserscan"

# Install foxglove bridge
RUN apt install -y ros-$ROS_DISTRO-foxglove-bridge

WORKDIR /ros2_ws
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /thirdparty/ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]