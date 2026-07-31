# Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _dwm1001_serial as dwm1001_serial  # noqa: E402

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _make_node(context, *args, **kwargs):
    dwm_config_path = LaunchConfiguration('dwm_config').perform(context)

    requested = dwm1001_serial.read_serial_port_from_yaml(dwm_config_path)
    serial_port, warning = dwm1001_serial.resolve_requested_or_auto(requested)

    actions = []
    if warning:
        actions.append(LogInfo(msg=f"[active_node.launch] {warning}"))

    # All other parameters (tag_id, samples, ...) come from dwm_config as-is;
    # this override only ever touches serial_port.
    actions.append(Node(
        package="dwm1001_driver",
        executable="active_tag",
        parameters=[dwm_config_path, {"serial_port": serial_port}],
    ))
    return actions


def generate_launch_description() -> LaunchDescription:

    default_dwm_config_path = PathJoinSubstitution([FindPackageShare('dwm1001_launch'),
                                                     'config',
                                                     'default_active.yaml'])
    dwm_config_launch_arg = DeclareLaunchArgument(
        'dwm_config',
        default_value=default_dwm_config_path,
        description='Configuration file for the active node'
    )

    return LaunchDescription([
        dwm_config_launch_arg,
        OpaqueFunction(function=_make_node),
    ])
