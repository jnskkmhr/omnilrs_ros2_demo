from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    joystick_config = os.path.join(
        get_package_share_directory('robot_controller'),
        'config',
        'joystick.yaml'
        )
    
    ns = LaunchConfiguration('namespace')
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )

    joy_node = Node(
            package='joy_linux', 
            namespace=ns, 
            executable='joy_linux_node', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'error'], 
            name="joy", 
        )
    
    teleop_node = Node(
            package='robot_controller', 
            namespace=ns,
            executable='teleop_joy', 
            output='screen', 
            arguments=['--ros-args', '--log-level', 'error'], 
            name="joystick_handler",
            parameters=[joystick_config],
            )
    
    ld.add_action(ns_arg)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld