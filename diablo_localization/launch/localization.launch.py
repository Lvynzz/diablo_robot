from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    share = get_package_share_directory("diablo_localization")
    default_params = os.path.join(share, "config", "diablo_ekf.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("imu_parent_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("imu_frame", default_value="diablo_robot"),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("reset_topic", default_value="/diablo/reset_pose"),
        DeclareLaunchArgument("reset_service", default_value="/diablo/reset_odom"),
        DeclareLaunchArgument(
            "set_pose_service", default_value="/diablo_ekf_filter/set_pose"
        ),
        DeclareLaunchArgument("reset_frame", default_value="odom"),
        DeclareLaunchArgument(
            "stop_cmd_topic",
            default_value="/diablo_base_controller/cmd_vel_unstamped",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="diablo_ekf_filter",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    )
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="diablo_base_to_imu",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("imu_x"),
                "--y", LaunchConfiguration("imu_y"),
                "--z", LaunchConfiguration("imu_z"),
                "--yaw", LaunchConfiguration("imu_yaw"),
                "--pitch", LaunchConfiguration("imu_pitch"),
                "--roll", LaunchConfiguration("imu_roll"),
                "--frame-id", LaunchConfiguration("imu_parent_frame"),
                "--child-frame-id", LaunchConfiguration("imu_frame"),
            ],
        ),
        Node(
            package="diablo_localization",
            executable="reset_pose",
            name="diablo_pose_reset",
            output="screen",
            parameters=[{
                "reset_topic": LaunchConfiguration("reset_topic"),
                "reset_service": LaunchConfiguration("reset_service"),
                "set_pose_service": LaunchConfiguration("set_pose_service"),
                "reset_frame": LaunchConfiguration("reset_frame"),
                "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
            }],
        ),
    ])
