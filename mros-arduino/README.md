# Motor controller

In this work, we use **micro-ros-arduino** to run ROS2 program on ESP32 MCU. \
**micro-ros-arduino** is arduino library of micro-ros. \
In addition to **micro-ros-arduino**, we also use **micro-ros-agent** to communicate between rclc (on MCU) and ROS2 FastDDS (Humble host computer).

## General system information
MCU: ESP-WROOM-32: [link](https://www.amazon.co.jp/WayinTop-ESP32%E9%96%8B%E7%99%BA%E3%83%9C%E3%83%BC%E3%83%89-BLE%E3%83%A2%E3%82%B8%E3%83%A5%E3%83%BC%E3%83%AB-ESP-WROOM-32%E5%AE%9F%E8%A3%85%E6%B8%88%E3%81%BF-%E5%B0%82%E7%94%A8USB%E3%82%B1%E3%83%BC%E3%83%96%E3%83%AB%E4%BB%98%E3%81%8D/dp/B086QKRY25/ref=sr_1_1_sspa?__mk_ja_JP=%E3%82%AB%E3%82%BF%E3%82%AB%E3%83%8A&crid=3CK8F8NSJ2V0Y&keywords=esp32&qid=1701748503&sprefix=esp32%2Caps%2C178&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) \
Arduino library: esp32-Dev-Module

## Setup

### 0. Configure Maxon motor parameter in ESCON-studio
Input motor parameter listed in Data Sheet, asked by prompt in ESCON-studio. 
You also need to set important info:
- Number of DigitalI/O (only 1 is necessary)
- Number of AnalogI/O (only 1 is necessary)
- Motor control mode (open-loop speed control, closed-loop speed control, open-loop current control)
- Ratio of analogIn voltage to RPM (if you are using speed control)

### 1. ESP32 in Arduino IDE
Refer to this [site](https://interface.cqpub.co.jp/esp32-arduino-ide-2/)

### 2. micro-ros-arduino
Download zip file from [here](https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.7-humble) and add it to Arduino IDE. 

### 3. micro-ros-agent
Use docker to run this.
```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
```
While running micro-ros-agent, esp32 can publish/subscribe topic in ROS2 DDS. 

If above does not run, make sure you grant permission to `/dev/ttyUSB0`. To do this, run the following.
```
sudo chown $USER /dev/ttyUSB0
```