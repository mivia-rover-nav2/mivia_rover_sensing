import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Stable device symlink created by mivia_rover_sensing/udev/99-xsens-mti.rules
_XSENS_UDEV_SYMLINK = '/dev/xsens_imu'


def _make_xsens_node(context, *args, **kwargs):
    """
    Prefer the fixed, udev-pinned device path. If the rule isn't installed
    (or the symlink hasn't appeared yet for some other reason), fall back to
    the driver's own device scan instead of failing outright - this only
    works if exactly one Xsens-compatible serial device is connected, but
    that's still better than a hard failure.
    """
    parameters_file_path = str(
        Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node.yaml')
    )

    actions = []
    if os.path.exists(_XSENS_UDEV_SYMLINK):
        overrides = {'scan_for_devices': False, 'port': _XSENS_UDEV_SYMLINK, 'baudrate': 921600}
    else:
        overrides = {'scan_for_devices': True}
        actions.append(LogInfo(msg=(
            f"[xsens_mti_node.launch] '{_XSENS_UDEV_SYMLINK}' not found (udev rule not installed, "
            "or sensor unplugged). Falling back to scan_for_devices=true - this only works if "
            "exactly one Xsens-compatible serial device is connected. To fix: "
            "sudo <path-to>/mivia_rover_sensing/udev/install_udev_rules.sh"
        )))

    actions.append(Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        # Later entries override earlier ones for the same key: the yaml
        # provides everything else, `overrides` only ever touches the
        # device-selection keys above.
        parameters=[parameters_file_path, overrides],
    ))
    return actions


def generate_launch_description():

    ld = LaunchDescription()

    # Set environment variables to control logging behavior
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(OpaqueFunction(function=_make_xsens_node))

    return ld
