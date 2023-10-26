#include <rclcpp/rclcpp.hpp>
#include "moonraker_controller/wheel_driver.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelDriver>("diff_driver"));
    rclcpp::shutdown();
    return 0;
}