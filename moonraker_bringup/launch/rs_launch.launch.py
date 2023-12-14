import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # get path to packages
    rs2_workspace = get_package_share_directory('realsense2_camera')

    # register configuration params
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_infra1 = LaunchConfiguration('enable_infra1')
    enable_infra2 = LaunchConfiguration('enable_infra2')
    enable_gyro = LaunchConfiguration('enable_gyro')
    enable_accel = LaunchConfiguration('enable_accel')

    # define parameter configuration launch
    color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Use color stream'
    )
    depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Use depth stream'
    )
    infra1_arg = DeclareLaunchArgument(
        'enable_infra1',
        default_value='false',
        description='Use infra1 stream'
    )
    infra2_arg = DeclareLaunchArgument(
        'enable_infra2',
        default_value='false',
        description='Use infra2 stream'
    )
    gyro_arg = DeclareLaunchArgument(
        'enable_gyro',
        default_value='false',
        description='Use gyro stream'
    )
    accel_arg = DeclareLaunchArgument(
        'enable_accel',
        default_value='false',
        description='Use accel stream'
    )

    launch_rs = os.path.join(rs2_workspace, 'launch', 'rs_launch.py')


    rs2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_rs), 
        launch_arguments=[
            ('enable_color', enable_color),
            ('enable_depth', enable_depth),
            ('enable_infra1', enable_infra1),
            ('enable_infra2', enable_infra2),
            ('enable_gyro', enable_gyro),
            ('enable_accel', enable_accel)
                ]
    )

    # add launch action to launch description
    ld = LaunchDescription()
    ld.add_action(color_arg)
    ld.add_action(depth_arg)
    ld.add_action(infra1_arg)
    ld.add_action(infra2_arg)
    ld.add_action(gyro_arg)
    ld.add_action(accel_arg)
    ld.add_action(rs2_launch)
    return ld