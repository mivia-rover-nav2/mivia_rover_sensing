#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    """

    # --- Launch arguments ---
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/serial/by-id/usb-SEGGER_J-Link_000760176108-if00",
        description="Stable serial device path (prefer /dev/serial/by-id/...).",
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

    # --- Node ---
    node = Node(
        package="dwm1001_driver",
        executable="les_mode_tag",  
        name="dwm1001_les_mode",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "send_les_on_start": LaunchConfiguration("send_les_on_start"),
            "reconnect_enabled": LaunchConfiguration("reconnect_enabled"),
            "reconnect_initial_delay_s": LaunchConfiguration("reconnect_initial_delay_s"),
            "reconnect_max_delay_s": LaunchConfiguration("reconnect_max_delay_s"),
            "reconnect_backoff": LaunchConfiguration("reconnect_backoff"),
            "reconnect_timer_period_s": LaunchConfiguration("reconnect_timer_period_s"),
        }],
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
        node,
    ])
