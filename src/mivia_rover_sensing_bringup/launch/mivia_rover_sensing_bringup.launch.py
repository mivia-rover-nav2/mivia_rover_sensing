from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    enable_xsens = LaunchConfiguration("enable_xsens")
    enable_ntrip = LaunchConfiguration("enable_ntrip")

    declare_args = [
        DeclareLaunchArgument(
            "enable_xsens",
            default_value=TextSubstitution(text="true"),
            description="Enable the Xsens MTi IMU/GNSS driver.",
        ),
        DeclareLaunchArgument(
            "enable_ntrip",
            default_value=TextSubstitution(text="true"),
            description="Enable the NTRIP client (RTK corrections for the Xsens GNSS receiver).",
        ),
    ]

    zed_launch_file_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed2_camera.launch.py'
    )
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file_path),
        launch_arguments={
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_cam_imu_transf':'true',
            'publish_imu_raw':'true',
            'publish_imu_tf':'true',
            'publish_mag':'true'
        }.items()
        )

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

    xsens_launch_file_path = os.path.join(
        get_package_share_directory('xsens_mti_ros2_driver'),
        'launch',
        'xsens_mti_node.launch.py'
    )

    xsens_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsens_launch_file_path),
        condition=IfCondition(enable_xsens),
    )

    ntrip_launch_file_path = os.path.join(
        get_package_share_directory('ntrip'),
        'launch',
        'ntrip_launch.py'
    )

    ntrip_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ntrip_launch_file_path),
        condition=IfCondition(enable_ntrip),
    )

    return LaunchDescription(
        declare_args + [
            zed_wrapper,
            dwm1001_wrapper,
            xsens_driver,
            ntrip_client,
        ]
    )
