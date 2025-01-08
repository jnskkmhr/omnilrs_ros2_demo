import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove Studio to listen on',
    )
    send_buffer_limit_arg = DeclareLaunchArgument(
        'send_buffer_limit',
        default_value='50000000000',
        description='Buffer limit for Foxglove Studio',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for nodes'
    )

    # Get launch configurations
    port = LaunchConfiguration('port')
    buffer_limit = LaunchConfiguration('send_buffer_limit')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node for converting encoding of camera messages from IsaacSim
    depth_cam_encoding_node = Node(
        package='vis_tool',
        executable='depth_encoding.py',
        name='depth_encoding',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',  # Ensure logs are visible
    )

    # Node for foxglove_bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': port,
            'send_buffer_limit': buffer_limit,
            'use_sim_time': use_sim_time
        }],
        arguments=['--port', port],
        output='screen'  # Ensure logs are visible
    )

    return LaunchDescription([
        # Launch arguments
        port_arg,
        send_buffer_limit_arg,
        use_sim_time_arg,

        # Nodes
        depth_cam_encoding_node,
        foxglove_node,
    ])
