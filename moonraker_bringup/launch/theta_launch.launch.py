from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package='theta_driver', 
            executable='theta_driver_node', 
            output='screen',
        )
    )
    return ld