#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "robot_controller/wheel_driver.hpp"

#define PI 3.141592

WheelDriver::WheelDriver(std::string node_name, std::string drive_model)
: Node(node_name){
  pub_throttle = this->create_publisher<std_msgs::msg::Float32MultiArray>
  ("throttle", 10);
  sub_twist = this->create_subscription<geometry_msgs::msg::Twist>
  ("cmd_vel", 10, std::bind(&WheelDriver::callback, this, std::placeholders::_1));
  _drive_model = drive_model; //make argument drive_model instance variable

  // declare ros parameters 
  this->declare_parameter("max_velocity", 0.3);
  this->declare_parameter("radius", 0.09);
  this->declare_parameter("base", 0.335);
}

void WheelDriver::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float linear_velocity = msg->linear.x;
  float angular_velocity = msg->angular.z;
  // float omega_to_rpm = 60/(2*PI);

  if(_drive_model == "diff_drive"){
    // get ros parameter
    float max_velocity = this->get_parameter("max_velocity").as_double();
    float radius = this->get_parameter("radius").as_double();
    float base = this->get_parameter("base").as_double();

    // diffrential controller
    float omega_max = max_velocity/radius;
    float omega_l = (linear_velocity+angular_velocity*(base/2))/radius;
    float omega_r = (linear_velocity-angular_velocity*(base/2))/radius;
    float throttle_l = omega_l/omega_max;
    float throttle_r = omega_r/omega_max;
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