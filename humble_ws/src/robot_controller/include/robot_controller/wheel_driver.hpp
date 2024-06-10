#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>


class WheelDriver : public rclcpp::Node
{
    public:
        WheelDriver(std::string node_name, std::string drive_model);
        std::string _drive_model;
    private:
        void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_throttle;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
};