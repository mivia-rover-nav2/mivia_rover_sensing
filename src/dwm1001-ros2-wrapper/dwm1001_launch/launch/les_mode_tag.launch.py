#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _dwm1001_serial as dwm1001_serial  # noqa: E402

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _make_node(context, *args, **kwargs):
    requested = LaunchConfiguration("serial_port").perform(context)
    serial_port, warning = dwm1001_serial.resolve_requested_or_auto(requested)

    actions = []
    if warning:
        actions.append(LogInfo(msg=f"[les_mode_tag.launch] {warning}"))

    actions.append(Node(
        package="dwm1001_driver",
        executable="les_mode_tag",
        name="dwm1001_les_mode",
        output="screen",
        parameters=[{
            "serial_port": serial_port,
            "baudrate": LaunchConfiguration("baudrate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "send_les_on_start": LaunchConfiguration("send_les_on_start"),
            "reconnect_enabled": LaunchConfiguration("reconnect_enabled"),
            "reconnect_initial_delay_s": LaunchConfiguration("reconnect_initial_delay_s"),
            "reconnect_max_delay_s": LaunchConfiguration("reconnect_max_delay_s"),
            "reconnect_backoff": LaunchConfiguration("reconnect_backoff"),
            "reconnect_timer_period_s": LaunchConfiguration("reconnect_timer_period_s"),
        }],
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Launch file for DWM1001LesPublisher (LES mode with auto-reconnection).

    Exposes all node parameters as launch arguments so you can override them from CLI, e.g.:

      ros2 launch dwm1001_driver dwm1001_les_mode.launch.py \
        serial_port:=/dev/serial/by-id/usb-XXX \
        baudrate:=115200 \
        frame_id:=dwm1001 \
        send_les_on_start:=true \
        reconnect_enabled:=true \
        reconnect_initial_delay_s:=0.5 \
        reconnect_max_delay_s:=10.0 \
        reconnect_backoff:=1.6 \
        reconnect_timer_period_s:=0.2

    Serial port resolution (udev symlink -> by-id -> raw fallback) is shared
    with other dwm1001_launch launch files via _dwm1001_serial.py.
    """

    # --- Launch arguments ---
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value=dwm1001_serial.AUTO,
        description=(
            "Serial device path, or 'auto' (default) to resolve it automatically: prefers "
            f"{dwm1001_serial.UDEV_SYMLINK} (udev rule in "
            "mivia_rover_sensing/udev/98-dwm1001-uwb.rules), falls back to "
            f"{dwm1001_serial.BY_ID_GLOB}, then to {dwm1001_serial.RAW_FALLBACK} as a last resort."
        ),
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baudrate for DWM1001.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="dwm1001",
        description="Frame id used in published messages.",
    )
    send_les_on_start_arg = DeclareLaunchArgument(
        "send_les_on_start",
        default_value="true",
        description="Send 'les' command on startup (after reset+shell) to enable LES stream.",
    )

    reconnect_enabled_arg = DeclareLaunchArgument(
        "reconnect_enabled",
        default_value="true",
        description="Enable auto-reconnect on serial failures/disconnections.",
    )
    reconnect_initial_delay_s_arg = DeclareLaunchArgument(
        "reconnect_initial_delay_s",
        default_value="0.5",
        description="Initial reconnect delay (seconds).",
    )
    reconnect_max_delay_s_arg = DeclareLaunchArgument(
        "reconnect_max_delay_s",
        default_value="10.0",
        description="Maximum reconnect delay (seconds).",
    )
    reconnect_backoff_arg = DeclareLaunchArgument(
        "reconnect_backoff",
        default_value="1.6",
        description="Backoff multiplier after each failed reconnect attempt.",
    )
    reconnect_timer_period_s_arg = DeclareLaunchArgument(
        "reconnect_timer_period_s",
        default_value="0.2",
        description="Reconnect watchdog timer period (seconds).",
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        frame_id_arg,
        send_les_on_start_arg,
        reconnect_enabled_arg,
        reconnect_initial_delay_s_arg,
        reconnect_max_delay_s_arg,
        reconnect_backoff_arg,
        reconnect_timer_period_s_arg,
        OpaqueFunction(function=_make_node),
    ])
