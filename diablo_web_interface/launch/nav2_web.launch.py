import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_map_file(bringup_share):
    """Use the map selected by the HMI for the combined Nav2 launch."""
    candidates = [Path(bringup_share) / "map"]
    for workspace_root in Path(bringup_share).parents:
        candidates.append(workspace_root / "src" / "diablo_bringup" / "map")
    for directory in candidates:
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
    return str(Path(bringup_share) / "map" / "empty.yaml")


def generate_launch_description():
    share = get_package_share_directory("diablo_web_interface")
    bringup_share = get_package_share_directory("diablo_bringup")
    web_launch = os.path.join(share, "launch", "web_interface.launch.py")
    navigation_launch = os.path.join(share, "launch", "navigation.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=_default_map_file(bringup_share)),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8000"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("odom_topic", default_value="/diablo/odometry"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "reset_encoder_service", default_value="/diablo/reset_encoder"
        ),
        DeclareLaunchArgument("lidar_start_service", default_value="/start_motor"),
        DeclareLaunchArgument("lidar_start_service_type", default_value="empty"),
        DeclareLaunchArgument("lidar_stop_service", default_value="/stop_motor"),
        DeclareLaunchArgument("lidar_stop_service_type", default_value="empty"),
        DeclareLaunchArgument("diablo_start_command", default_value="ros2 run diablo_ctrl diablo_ctrl_node"),
        DeclareLaunchArgument("lidar_start_command", default_value=""),
        DeclareLaunchArgument(
            "dynamixel_start_command",
            default_value="ros2 launch diablo_bringup six_joint_move.launch.py",
        ),
        DeclareLaunchArgument("hardware_log_directory", default_value="/tmp"),
        DeclareLaunchArgument("localization_start_command", default_value=""),
        DeclareLaunchArgument("navigation_start_command", default_value=""),
        DeclareLaunchArgument("mapping_start_command", default_value=""),
        DeclareLaunchArgument("maps_dir", default_value=""),
        DeclareLaunchArgument(
            "enable_wheel_odom",
            default_value="true",
            description="Run one standalone wheel odometry node from the web launch",
        ),
        DeclareLaunchArgument("wheel_radius", default_value="0.093"),
        DeclareLaunchArgument("track_width", default_value="0.475"),
        DeclareLaunchArgument("motor_topic", default_value="/diablo/sensor/Motors"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch),
            launch_arguments={
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "reset_encoder_service": LaunchConfiguration("reset_encoder_service"),
                "lidar_start_service": LaunchConfiguration("lidar_start_service"),
                "lidar_start_service_type": LaunchConfiguration("lidar_start_service_type"),
                "lidar_stop_service": LaunchConfiguration("lidar_stop_service"),
                "lidar_stop_service_type": LaunchConfiguration("lidar_stop_service_type"),
                "diablo_start_command": LaunchConfiguration("diablo_start_command"),
                "lidar_start_command": LaunchConfiguration("lidar_start_command"),
                "dynamixel_start_command": LaunchConfiguration("dynamixel_start_command"),
                "hardware_log_directory": LaunchConfiguration("hardware_log_directory"),
                "localization_start_command": LaunchConfiguration("localization_start_command"),
                "navigation_start_command": LaunchConfiguration("navigation_start_command"),
                "mapping_start_command": LaunchConfiguration("mapping_start_command"),
                "maps_dir": LaunchConfiguration("maps_dir"),
                "enable_mux": "true",
                "enable_wheel_odom": LaunchConfiguration("enable_wheel_odom"),
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "track_width": LaunchConfiguration("track_width"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                # web_interface.launch.py owns the single odometry instance.
                "enable_wheel_odom": "false",
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "track_width": LaunchConfiguration("track_width"),
                "motor_topic": LaunchConfiguration("motor_topic"),
                "enable_mux": "false",
            }.items(),
        ),
    ])
