from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package='joy_linux', 
            executable='joy_linux_node', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'error']
        )
    )
    ld.add_action(
        Node(
            package='moonraker_controller', 
            executable='teleop_joy', 
            output='screen', 
            arguments=['--ros-args', '--log-level', 'error']
            )
    )
    ld.add_action(
        Node(
            package='moonraker_controller', 
            executable='diff_controller', 
            output='screen', 
            arguments=['--ros-args', '--log-level', 'error']
            )
    )
    return ld