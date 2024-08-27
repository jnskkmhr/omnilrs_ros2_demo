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
        libspdlog-dev \
        libsuitesparse-dev \
        qtdeclarative5-dev \
        qt5-qmake \
        libqglviewer-dev-qt5 \
        libopencv-dev \
        libpcl-dev \
        libpcl-conversions-dev \
        libomp-dev \
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-joy-linux \
        ros-$ROS_DISTRO-image-geometry && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-pcl-*

# Install nav2 and slamtoolbox
RUN apt-get update && apt-get install -y \
ros-${ROS_DISTRO}-navigation2 \
ros-${ROS_DISTRO}-nav2-bringup \
ros-${ROS_DISTRO}-slam-toolbox \
ros-${ROS_DISTRO}-spatio-temporal-voxel-layer

# Install foxglove bridge
RUN apt install -y ros-$ROS_DISTRO-foxglove-bridge

# Install octomap
RUN apt-get install -y ros-${ROS_DISTRO}-octomap-mapping

WORKDIR /ros2_ws
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]