#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "robot_controller/joystick_handler.hpp"

#define DRIVE_AXIS 1
#define ROT_AXIS 3

JoystickHandler::JoystickHandler(std::string node_name)
: Node(node_name) {
    pub_twist = this->create_publisher<geometry_msgs::msg::Twist>
    ("cmd_vel", 10);
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>
    ("joy", 10, std::bind(&JoystickHandler::callback, this, std::placeholders::_1));

    //declare rosparameter
    this->declare_parameter("max_velocity", 0.3);
    this->declare_parameter("max_omega", 0.2);
}

void JoystickHandler::callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float max_velocity = this->get_parameter("max_velocity").as_double();
    float max_omega = this->get_parameter("max_omega").as_double();
    float drive_throttle = msg->axes[DRIVE_AXIS];
    float rot_throttle = msg->axes[ROT_AXIS];
    float linear_velocity = drive_throttle * max_velocity;
    float angular_velocity = rot_throttle * max_omega;

    auto pub_msg = geometry_msgs::msg::Twist();
    pub_msg.linear.x = linear_velocity;
    pub_msg.linear.y = 0;
    pub_msg.linear.z = 0;
    pub_msg.angular.x = 0;
    pub_msg.angular.y = 0;
    pub_msg.angular.z = angular_velocity;
    pub_twist->publish(pub_msg);
}