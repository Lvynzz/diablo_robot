from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8000"),
        DeclareLaunchArgument(
            "manual_cmd_topic", default_value="/diablo/MotionCmd/manual"
        ),
        DeclareLaunchArgument(
            "control_mode_topic", default_value="/diablo/control_mode"
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "reset_encoder_service", default_value="/diablo/reset_encoder"
        ),
        DeclareLaunchArgument("lidar_start_service", default_value="/lidar/start"),
        DeclareLaunchArgument(
            "diablo_start_command", default_value="ros2 run diablo_ctrl diablo_ctrl_node"
        ),
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
        DeclareLaunchArgument("enable_mux", default_value="true"),

        SetEnvironmentVariable("DIABLO_WEB_HOST", LaunchConfiguration("host")),
        SetEnvironmentVariable("DIABLO_WEB_PORT", LaunchConfiguration("port")),

        Node(
            package="diablo_web_interface",
            executable="web_node",
            name="diablo_web_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "manual_cmd_topic": LaunchConfiguration("manual_cmd_topic"),
                "control_mode_topic": LaunchConfiguration("control_mode_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "map_frame": LaunchConfiguration("map_frame"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "reset_encoder_service": LaunchConfiguration("reset_encoder_service"),
                "lidar_start_service": LaunchConfiguration("lidar_start_service"),
                "diablo_start_command": LaunchConfiguration("diablo_start_command"),
                "lidar_start_command": LaunchConfiguration("lidar_start_command"),
                "dynamixel_start_command": LaunchConfiguration("dynamixel_start_command"),
                "hardware_log_directory": LaunchConfiguration("hardware_log_directory"),
                "localization_start_command": LaunchConfiguration("localization_start_command"),
                "navigation_start_command": LaunchConfiguration("navigation_start_command"),
                "mapping_start_command": LaunchConfiguration("mapping_start_command"),
                "maps_dir": LaunchConfiguration("maps_dir"),
            }],
        ),
        Node(
            package="diablo_web_interface",
            executable="motion_cmd_mux",
            name="diablo_motion_cmd_mux",
            output="screen",
            parameters=[{
                "manual_topic": LaunchConfiguration("manual_cmd_topic"),
                "control_mode_topic": LaunchConfiguration("control_mode_topic"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_mux")),
        ),
    ])
