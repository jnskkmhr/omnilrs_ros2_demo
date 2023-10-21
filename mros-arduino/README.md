# Micro-ROS-Arduino

Micro-ros-arduino is arduino library of micro-ros. We use this package for communication between MCU and computer with ROS2. 

## General info
MCU: esp32 \
Arduino library: esp32-Dev-Module

## Setup

### 0. esp32 in Arduino IDE
Refer to this [site](https://interface.cqpub.co.jp/esp32-arduino-ide-2/)

### 1. micro-ros-arduino
Download zip file from [here](https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.5-foxy) and add it to Arduino IDE. 

### 2. micro-ros-agent
Use docker for running this.
```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:foxy serial --dev /dev/ttyUSB0 -v6
```
While running micro-ros-agent, esp32 can publish/subscribe topic in ROS2 DDS. 

If above does not run, make sure you have permission to `/dev/ttyUSB0`. To ensure that, do the following.
```
sudo chown $USER /dev/ttyUSB0
```