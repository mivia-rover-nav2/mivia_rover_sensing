# Copyright ...
# (stesso header legale che usi normalmente)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Argomento per selezionare il file di config del visualizer
    anchors_config = LaunchConfiguration('anchors_config')

    default_anchors_config = PathJoinSubstitution([
        FindPackageShare('dwm1001_launch'),
        'config',
        'anchors.yaml'
    ])

    anchors_config_arg = DeclareLaunchArgument(
        'anchors_config',
        default_value=default_anchors_config,
        description='YAML with anchor_visualizer parameters'
    )

    # Nodo di visualizzazione (package separato)
    anchor_visualizer = Node(
        package='dwm1001_visualization',
        executable='anchor_visualizer',
        name='anchor_visualizer',
        parameters=[anchors_config],
        output='screen'
    )

    static_tf = Node(
         package='tf2_ros',
         executable='static_transform_publisher',
         name='static_map_to_dwm1001',
         arguments=['0', '0', '0', '0', '0', '0', 'map', 'dwm1001'],
         output='screen'
     )

    return LaunchDescription([
        anchors_config_arg,
        anchor_visualizer,
        # static_tf
    ])
