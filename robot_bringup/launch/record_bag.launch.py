import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    save_path = os.path.join(os.path.expanduser('~'), 'ros2_bag', f'{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}')
    bag_cmd1 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', save_path, '--all'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(bag_cmd1)
    return ld