import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_workspace = get_package_share_directory('nav2_bringup')

    launch_nav2 = os.path.join(nav2_workspace, 'launch', 'nav2_bringup_launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav2),
    )

    ld = LaunchDescription()
    ld.add_action(nav2_launch)
    return ld