#include <rclcpp/rclcpp.hpp>
#include "robot_controller/wheel_driver.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelDriver>("wheel_drive", "diff_drive"));
    rclcpp::shutdown();
    return 0;
}