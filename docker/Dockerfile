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
ros-${ROS_DISTRO}-slam-toolbox

# Install foxglove bridge
RUN apt install -y ros-$ROS_DISTRO-foxglove-bridge

# Build other thirdparty packages from source

# g2o
WORKDIR /thirdparty
RUN git clone https://github.com/RainerKuemmerle/g2o.git
WORKDIR /thirdparty/g2o
RUN mkdir build && \
cd build && cmake .. && \
make && make install

# lidarslam_ros2 and pointcloud_to_grid
WORKDIR /thirdparty/ros2_ws/src
RUN git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2.git
RUN git clone https://github.com/jkk-research/pointcloud_to_grid -b ros2
WORKDIR /thirdparty/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && \
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

WORKDIR /ros2_ws
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /thirdparty/ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]