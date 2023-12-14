import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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
    usb_port_id = LaunchConfiguration('usb_port_id')
    camera_name = LaunchConfiguration('camera_name')
    camera_ns = LaunchConfiguration('camera_namespace')

    # define parameter configuration launch
    color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='false',
        description='Use color stream'
    )
    depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Use depth stream'
    )
    infra1_arg = DeclareLaunchArgument(
        'enable_infra1',
        default_value='true',
        description='Use infra1 stream'
    )
    infra2_arg = DeclareLaunchArgument(
        'enable_infra2',
        default_value='true',
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
    usb_port_id_arg = DeclareLaunchArgument(
        'usb_port_id',
        default_value='',
        description='USB port ID'
    )
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name'
    )
    camera_ns_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='Camera namespace'
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
            ('enable_accel', enable_accel), 
            ('usb_port_id', usb_port_id), 
            ('camera_name', camera_name),
            ('camera_namespace', camera_ns),
                ]
    )

    # ros2 set param /camera/camera depth_module.emitter_enabled 0 in launch file
    # get value of cameraname and camera namespace and save it in a variable

    disable_laser_emitter1_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set',
            '/d435i/d435i',
            'depth_module.emitter_enabled', '0'
        ],
        shell=False
    )

    disable_laser_emitter2_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set',
            '/d455/d455',
            'depth_module.emitter_enabled', '0'
        ],
        shell=False
    )

    # add launch action to launch description
    ld = LaunchDescription()
    ld.add_action(color_arg)
    ld.add_action(depth_arg)
    ld.add_action(infra1_arg)
    ld.add_action(infra2_arg)
    ld.add_action(gyro_arg)
    ld.add_action(accel_arg)
    ld.add_action(usb_port_id_arg)
    ld.add_action(camera_name_arg)
    ld.add_action(camera_ns_arg)
    ld.add_action(rs2_launch)
    ld.add_action(disable_laser_emitter1_cmd)
    ld.add_action(disable_laser_emitter2_cmd)
    return ld