#!/usr/bin/env python3
"""Standalone AMCL localization launch for the Diablo web HMI.

The web launch already owns wheel odometry and the LiDAR TF.  This file only
starts the map server, AMCL, and their lifecycle manager so it can be toggled
independently from mapping and the full Nav2 navigation stack.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_map_file(web_share):
    """Choose a real bringup map when one is installed, else bundled empty map."""
    candidates = []
    bringup_share = None
    try:
        bringup_share = Path(get_package_share_directory("diablo_bringup"))
        candidates.append(bringup_share / "map")
        # With --symlink-install, maps saved in the source checkout may not yet
        # be installed.  Look for the sibling source package as a fallback.
        for workspace_root in bringup_share.parents:
            candidates.append(workspace_root / "src" / "diablo_bringup" / "map")
    except Exception:
        pass
    candidates.append(Path(web_share) / "maps")
    for directory in candidates:
        # Match the AMR HMI behavior: the map selected in the web UI is the
        # map consumed by the next localization launch.  Ignore stale markers
        # and continue with the normal deterministic fallback.
        marker = directory / ".selected_localization_map.txt"
        try:
            selected = marker.read_text(encoding="utf-8").strip()
        except OSError:
            selected = ""
        if selected:
            selected_path = Path(selected).expanduser()
            if selected_path.suffix.lower() == ".pgm":
                selected_path = selected_path.with_suffix(".yaml")
            # The install-space map files may be symlinks into the source
            # checkout.  Resolve the marker by basename inside each candidate
            # directory; resolving the file first makes the old parent check
            # reject valid install-space symlinks and fall back to empty.yaml.
            selected_path = directory / selected_path.name
            try:
                if selected_path.is_file():
                    return str(selected_path)
            except OSError:
                pass
        try:
            yaml_files = sorted(directory.glob("*.yaml"))
        except OSError:
            yaml_files = []
        if yaml_files:
            return str(yaml_files[0])
    if bringup_share is not None:
        return str(bringup_share / "map" / "empty.yaml")
    return str(Path(web_share) / "maps" / "empty.yaml")


def generate_launch_description():
    web_share = get_package_share_directory("diablo_web_interface")
    bringup_share = get_package_share_directory("diablo_bringup")
    default_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = _default_map_file(web_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map_file")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map_file", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("odom_topic", default_value="/diablo/odometry"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="false",
            description="Use launch-provided pose instead of waiting for /initialpose.",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "yaml_filename": ParameterValue(map_file, value_type=str),
                },
            ],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "base_frame_id": ParameterValue(base_frame, value_type=str),
                    "odom_frame_id": ParameterValue(odom_frame, value_type=str),
                    "scan_topic": ParameterValue(scan_topic, value_type=str),
                    "set_initial_pose": ParameterValue(
                        LaunchConfiguration("set_initial_pose"), value_type=bool
                    ),
                    "initial_pose.x": ParameterValue(
                        LaunchConfiguration("initial_pose_x"), value_type=float
                    ),
                    "initial_pose.y": ParameterValue(
                        LaunchConfiguration("initial_pose_y"), value_type=float
                    ),
                    "initial_pose.yaw": ParameterValue(
                        LaunchConfiguration("initial_pose_yaw"), value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                },
            ],
        ),
    ])
