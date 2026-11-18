from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    dwm1001_launch_file_path = os.path.join(
        get_package_share_directory('dwm1001_launch'),
        'launch',
        'les_mode_tag.launch.py'
    )

    dwm1001_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            dwm1001_launch_file_path
            )
    )

    return LaunchDescription([
        dwm1001_wrapper
    ])
