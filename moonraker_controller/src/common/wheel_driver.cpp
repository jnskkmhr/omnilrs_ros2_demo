#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "moonraker_controller/wheel_driver.hpp"

#define PI 3.14
#define MAX_RPM 50.0
#define RADIUS 0.09
#define BASE 0.335

WheelDriver::WheelDriver(std::string node_name, std::string drive_model)
: Node(node_name){
  pub_throttle = this->create_publisher<std_msgs::msg::Float32MultiArray>
  ("throttle", 10);
  sub_twist = this->create_subscription<geometry_msgs::msg::Twist>
  ("cmd_vel", 10, std::bind(&WheelDriver::callback, this, std::placeholders::_1));
  _drive_model = drive_model; //make argument drive_model instance variable
}

void WheelDriver::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float linear_velocity = msg->linear.x;
  float angular_velocity = msg->angular.z;
  float omega_to_rpm = 60/(2*PI);

  if(_drive_model == "diff_drive"){
    // diffrential controller
    float omega_l = (linear_velocity-angular_velocity*(BASE/2))/RADIUS;
    float omega_r = (linear_velocity+angular_velocity*(BASE/2))/RADIUS;
    float throttle_l = (omega_l*omega_to_rpm)/MAX_RPM;
    float throttle_r = (omega_r*omega_to_rpm)/MAX_RPM;
    auto pub_msg = std_msgs::msg::Float32MultiArray();
    pub_msg.data.resize(2);
    pub_msg.data[0] = throttle_l;
    pub_msg.data[1] = throttle_r;
    pub_throttle->publish(pub_msg);
  }
  else if (_drive_model == "torque_control"){
    // torque control via 
    float omega_l = (linear_velocity-angular_velocity*(BASE/2))/RADIUS;
    float omega_r = (linear_velocity+angular_velocity*(BASE/2))/RADIUS;
    float throttle_l = (omega_l*omega_to_rpm)/MAX_RPM;
    float throttle_r = (omega_r*omega_to_rpm)/MAX_RPM;
    auto pub_msg = std_msgs::msg::Float32MultiArray();
    pub_msg.data.resize(2);
    pub_msg.data[0] = throttle_l;
    pub_msg.data[1] = throttle_r;
    pub_throttle->publish(pub_msg);
  }
  else {
    RCLCPP_INFO_ONCE(this->get_logger(), "Invalid drive model");
  }
}