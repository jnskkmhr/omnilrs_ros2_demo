from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('moonraker_controller'),
        'config',
        'skid_steer.yaml'
        )
    
    ns = LaunchConfiguration('namespace')
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )

    controller_node = Node(
            package='moonraker_controller',
            namespace=ns,
            executable='diff_controller', 
            output='screen', 
            arguments=['--ros-args', '--log-level', 'error'], 
            name='diff_controller', 
            parameters=[config],
            )
    
    ld.add_action(ns_arg)
    ld.add_action(controller_node)

    return ld