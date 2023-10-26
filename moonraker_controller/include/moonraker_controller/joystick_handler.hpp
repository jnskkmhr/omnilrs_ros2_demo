#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>


class JoystickHandler : public rclcpp::Node
{
    public:
        JoystickHandler(std::string node_name);
    private:
        void callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
};