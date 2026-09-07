from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Keep this launch file standalone so it can also be used with another
    # odometry source during bench tests.
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_odom_topic", default_value="/diablo_base_controller/odom"
        ),
        DeclareLaunchArgument("output_odom_topic", default_value="/diablo/odometry"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("reset_topic", default_value="/diablo/reset_pose"),
        DeclareLaunchArgument("reset_service", default_value="/diablo/reset_odom"),
        DeclareLaunchArgument(
            "stop_cmd_topic",
            default_value="/diablo_base_controller/cmd_vel_unstamped",
        ),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("reset_on_start", default_value="false"),
        Node(
            package="diablo_localization",
            executable="local_odom",
            name="diablo_local_odom",
            output="screen",
            parameters=[{
                "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "reset_topic": LaunchConfiguration("reset_topic"),
                "reset_service": LaunchConfiguration("reset_service"),
                "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("publish_tf"), value_type=bool
                ),
                "reset_on_start": ParameterValue(
                    LaunchConfiguration("reset_on_start"), value_type=bool
                ),
            }],
        ),
    ])
