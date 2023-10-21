from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package='joy_linux', 
            node_executable='joy_linux_node', 
            output='screen',
        )
    )
    ld.add_action(
        Node(
            package='moonraker_teleop_joy', 
            node_executable='diff_drive_joy', 
            output='screen'
            )
    )
    return ld