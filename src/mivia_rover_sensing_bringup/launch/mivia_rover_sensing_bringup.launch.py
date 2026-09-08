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
    enable_velodyne = LaunchConfiguration("enable_velodyne")

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
        DeclareLaunchArgument(
            "enable_velodyne",
            default_value=TextSubstitution(text="true"),
            description="Enable the Velodyne VLP-16 LiDAR driver.",
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

    # --- Velodyne VLP-16 --------------------------------------------------
    # The stock velodyne launch files are not parameterizable, so the two
    # nodes are started here directly against our own override params
    # (config/velodyne_vlp16.yaml). The calibration file is resolved to the
    # VLP16db.yaml installed by velodyne_pointcloud.
    velodyne_params_file = os.path.join(
        get_package_share_directory('mivia_rover_sensing_bringup'),
        'config',
        'velodyne_vlp16.yaml'
    )
    velodyne_calibration_file = os.path.join(
        get_package_share_directory('velodyne_pointcloud'),
        'params',
        'VLP16db.yaml'
    )

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='both',
        parameters=[velodyne_params_file],
        condition=IfCondition(enable_velodyne),
    )

    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='both',
        parameters=[velodyne_params_file, {'calibration': velodyne_calibration_file}],
        condition=IfCondition(enable_velodyne),
    )

    return LaunchDescription(
        declare_args + [
            zed_wrapper,
            dwm1001_wrapper,
            xsens_driver,
            ntrip_client,
            velodyne_driver_node,
            velodyne_transform_node,
        ]
    )
