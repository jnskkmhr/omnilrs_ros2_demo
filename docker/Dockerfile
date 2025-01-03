FROM osrf/ros:humble-desktop
ENV ROS_DISTRO=humble

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
        python3-rosdep \
        python3-pip \
        ros-$ROS_DISTRO-joy-linux \
        ros-$ROS_DISTRO-image-geometry && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Check if rosdep needs to be initialized
RUN if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then \
        rosdep init; \
    fi \
    && rosdep update

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
# RUN apt-get install -y ros-${ROS_DISTRO}-octomap-mapping

COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /ros2_ws
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]