#include <rclcpp/rclcpp.hpp>
#include "moonraker_controller/joystick_handler.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickHandler>("joy_handler"));
    rclcpp::shutdown();
    return 0;
}