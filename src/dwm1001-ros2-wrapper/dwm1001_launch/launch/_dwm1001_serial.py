#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Shared DWM1001 serial-port resolution logic, reused by every dwm1001_launch
launch file that needs to open the tag's serial port (les_mode_tag.launch.py
today; active_node.launch.py / passive_node.launch.py if/when they need it).

Two call patterns are supported, matching the two ways dwm1001_launch launch
files get their parameters today:

1. Parameters exposed as individual launch arguments (les_mode_tag.launch.py):

    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import _dwm1001_serial as dwm1001_serial

    serial_port, warning = dwm1001_serial.resolve_requested_or_auto(
        LaunchConfiguration("serial_port").perform(context)
    )

2. Parameters loaded wholesale from a yaml config file (active_node.launch.py,
   passive_node.launch.py) - all *other* parameters (tag_id, samples, ...)
   stay in the yaml; only serial_port gets layered on top as an override:

    requested = dwm1001_serial.read_serial_port_from_yaml(yaml_path)
    serial_port, warning = dwm1001_serial.resolve_requested_or_auto(requested)
    node = Node(..., parameters=[yaml_path, {"serial_port": serial_port}])

Not installed/importable as a proper ROS package module (dwm1001_launch is
ament_cmake, not ament_python) - this relies on `install(DIRECTORY launch ...)`
copying this file alongside the launch files themselves, and on each launch
file adding its own directory to sys.path before importing it.
"""

import glob
import os

import yaml

# Stable device symlink created by mivia_rover_sensing/udev/98-dwm1001-uwb.rules
UDEV_SYMLINK = "/dev/dwm1001_uwb"

# Standard Ubuntu by-id path (auto-created for any USB-serial device, no custom
# rule needed - just tied to one specific unit's serial number and less readable).
BY_ID_GLOB = "/dev/serial/by-id/usb-SEGGER_J-Link_*-if00"

RAW_FALLBACK = "/dev/ttyACM0"

INSTALL_HINT = "sudo <path-to>/mivia_rover_sensing/udev/install_udev_rules.sh"

# Sentinel value meaning "resolve automatically" for a `serial_port` launch
# argument default. Any other value passed by the caller is used as-is.
AUTO = "auto"


def resolve_serial_port():
    """Returns (serial_port, warning_message_or_None)."""
    if os.path.exists(UDEV_SYMLINK):
        return UDEV_SYMLINK, None

    by_id_matches = sorted(glob.glob(BY_ID_GLOB))
    if len(by_id_matches) == 1:
        return by_id_matches[0], (
            f"'{UDEV_SYMLINK}' not found (udev rule not installed?). Falling back to "
            f"'{by_id_matches[0]}' (standard by-id path, still stable across reboots)."
        )
    if len(by_id_matches) > 1:
        return by_id_matches[0], (
            f"'{UDEV_SYMLINK}' not found, and {len(by_id_matches)} SEGGER J-Link by-id devices "
            f"are present {by_id_matches}. Picking '{by_id_matches[0]}' arbitrarily - install the "
            f"udev rule to remove this ambiguity: {INSTALL_HINT}"
        )
    return RAW_FALLBACK, (
        f"'{UDEV_SYMLINK}' not found and no SEGGER J-Link by-id device present either. Falling "
        f"back to raw '{RAW_FALLBACK}' - THIS WILL BE WRONG if USB enumeration order shifts. "
        f"To fix: {INSTALL_HINT}"
    )


def read_serial_port_from_yaml(yaml_path):
    """
    Reads the `serial_port` value out of a ROS 2 params yaml file (first
    node's `ros__parameters` block). Returns '' if the file can't be read/
    parsed, or the key isn't present - callers should treat that the same as
    the AUTO sentinel.
    """
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return ""

    for node_params in data.values():
        ros_params = (node_params or {}).get("ros__parameters", {}) or {}
        if "serial_port" in ros_params:
            return str(ros_params["serial_port"])
    return ""


def resolve_requested_or_auto(requested):
    """
    `requested` is the already-performed (string) value of a `serial_port`
    launch argument. Returns (serial_port, warning_message_or_None).

    If the caller passed an explicit path (anything other than empty/AUTO),
    it's returned unchanged - this is what makes `serial_port:=/explicit/path`
    overrides bypass auto-resolution entirely.
    """
    if requested and requested != AUTO:
        return requested, None
    return resolve_serial_port()
