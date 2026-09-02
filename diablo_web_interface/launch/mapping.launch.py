import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("diablo_web_interface")
    default_params = os.path.join(share, "config", "slam_toolbox.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("motor_topic", default_value="/diablo/sensor/Motors"),
        DeclareLaunchArgument("enable_wheel_odom", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("wheel_radius", default_value="0.105"),
        DeclareLaunchArgument("track_width", default_value="0.3751"),
        Node(
            package="diablo_web_interface",
            executable="wheel_odom",
            name="diablo_wheel_odom",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("motor_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "track_width": LaunchConfiguration("track_width"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_wheel_odom")),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "scan_topic": ParameterValue(
                        LaunchConfiguration("scan_topic"), value_type=str
                    ),
                },
            ],
        ),
    ])
