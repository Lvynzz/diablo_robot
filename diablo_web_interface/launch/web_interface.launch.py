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
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_topic", default_value="/diablo/odometry"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "reset_encoder_service", default_value="/diablo/reset_encoder"
        ),
        DeclareLaunchArgument("lidar_start_service", default_value="/start_motor"),
        DeclareLaunchArgument("lidar_start_service_type", default_value="empty"),
        DeclareLaunchArgument("lidar_stop_service", default_value="/stop_motor"),
        DeclareLaunchArgument("lidar_stop_service_type", default_value="empty"),
        DeclareLaunchArgument(
            "diablo_start_command",
            default_value=(
                "ros2 run diablo_ctrl diablo_ctrl_node "
                "--ros-args -p controller_port:=/dev/diablo_controller"
            ),
        ),
        DeclareLaunchArgument(
            "lidar_start_command",
            default_value=(
                "ros2 launch sllidar_ros2 sllidar_a2m7_launch.py "
                "serial_port:=/dev/rplidar frame_id:=laser"
            ),
        ),
        DeclareLaunchArgument(
            "dynamixel_start_command",
            default_value=(
                "ros2 launch diablo_full_body_moveit_config full_body_hardware.launch.py "
                "use_mock_hardware:=false upper_only:=true "
                "enable_arm_hardware:=true enable_hand_hardware:=false enable_base_hardware:=false "
                "arm_port_name:=/dev/u2d2_arm hand_port_name:=/dev/u2d2_hand "
                "baud_rate:=1000000 start_arm_controllers:=true start_base_controller:=false "
                "use_ekf:=false use_local_odom:=false start_move_group:=false"
            ),
        ),
        DeclareLaunchArgument("hardware_log_directory", default_value="/tmp"),
        DeclareLaunchArgument("hardware_feedback_timeout", default_value="15.0"),
        DeclareLaunchArgument(
            "localization_start_command",
            default_value="ros2 launch diablo_web_interface localization.launch.py",
        ),
        DeclareLaunchArgument(
            "navigation_start_command",
            default_value="ros2 launch diablo_web_interface navigation.launch.py",
        ),
        DeclareLaunchArgument(
            "mapping_start_command",
            default_value=(
                "ros2 launch diablo_web_interface mapping.launch.py "
                "enable_wheel_odom:=false scan_topic:=/scan"
            ),
        ),
        DeclareLaunchArgument("maps_dir", default_value=""),
        DeclareLaunchArgument("enable_mux", default_value="true"),
        DeclareLaunchArgument("enable_wheel_odom", default_value="true"),
        DeclareLaunchArgument("wheel_radius", default_value="0.093"),
        DeclareLaunchArgument("track_width", default_value="0.475"),
        DeclareLaunchArgument("max_wheel_delta", default_value="1.5"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.08"),
        DeclareLaunchArgument("lidar_z", default_value="0.17"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.141592653589793"),

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
                "map_topic": LaunchConfiguration("map_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "map_frame": LaunchConfiguration("map_frame"),
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
                "hardware_feedback_timeout": LaunchConfiguration("hardware_feedback_timeout"),
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
        Node(
            package="diablo_web_interface",
            executable="wheel_odom",
            name="diablo_wheel_odom",
            output="screen",
            parameters=[{
                "input_topic": "/diablo/sensor/Motors",
                "odom_topic": LaunchConfiguration("odom_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "track_width": LaunchConfiguration("track_width"),
                "max_wheel_delta": LaunchConfiguration("max_wheel_delta"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_wheel_odom")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="diablo_lidar_static_tf",
            output="screen",
            arguments=[
                LaunchConfiguration("lidar_x"),
                LaunchConfiguration("lidar_y"),
                LaunchConfiguration("lidar_z"),
                LaunchConfiguration("lidar_yaw"),
                "0.0",
                "0.0",
                LaunchConfiguration("base_frame"),
                "laser",
            ],
            condition=IfCondition(LaunchConfiguration("publish_lidar_tf")),
        ),
    ])
