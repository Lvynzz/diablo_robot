"""Start the Diablo driver, raw wheel odometry, and the single EKF.

The web HMI owns this launch through its DIABLO hardware button.  The wheel
node publishes only the private input ``/diablo_base_controller/odom``; Nav2,
AMCL, and the web UI consume the EKF output ``/odometry/filtered``.  Keeping
the raw source private avoids a second ``odom -> base`` TF publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_share = get_package_share_directory("diablo_localization")
    ekf_params = os.path.join(localization_share, "config", "diablo_ekf.yaml")
    localization_launch = os.path.join(
        localization_share, "launch", "localization.launch.py"
    )

    controller_port = LaunchConfiguration("controller_port")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    filtered_odom_topic = LaunchConfiguration("filtered_odom_topic")
    motor_topic = LaunchConfiguration("motor_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    wheel_radius = LaunchConfiguration("wheel_radius")
    track_width = LaunchConfiguration("track_width")
    left_direction = LaunchConfiguration("left_wheel_direction")
    right_direction = LaunchConfiguration("right_wheel_direction")
    use_revolutions = LaunchConfiguration("use_encoder_revolutions")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_port", default_value="/dev/diablo_controller"
        ),
        DeclareLaunchArgument(
            "raw_odom_topic", default_value="/diablo_base_controller/odom"
        ),
        DeclareLaunchArgument(
            "filtered_odom_topic", default_value="/odometry/filtered"
        ),
        DeclareLaunchArgument(
            "motor_topic", default_value="/diablo/sensor/Motors"
        ),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("wheel_radius", default_value="0.093"),
        DeclareLaunchArgument("track_width", default_value="0.475"),
        DeclareLaunchArgument("left_wheel_direction", default_value="1.0"),
        DeclareLaunchArgument("right_wheel_direction", default_value="1.0"),
        DeclareLaunchArgument("use_encoder_revolutions", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=ekf_params),
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
            "reset_position_service", default_value="/diablo/reset_position"
        ),
        DeclareLaunchArgument(
            "reset_orientation_service", default_value="/diablo/reset_orientation"
        ),
        DeclareLaunchArgument(
            "set_pose_service", default_value="/set_pose"
        ),
        DeclareLaunchArgument("reset_frame", default_value="odom"),
        DeclareLaunchArgument("stop_cmd_topic", default_value=""),
        Node(
            package="diablo_ctrl",
            executable="diablo_ctrl_node",
            name="diablo_ctrl_node",
            output="screen",
            parameters=[{
                "controller_port": controller_port,
                "imu_frame_id": LaunchConfiguration("imu_frame"),
            }],
        ),
        Node(
            package="diablo_localization",
            executable="raw_wheel_odom",
            name="diablo_raw_wheel_odom",
            output="screen",
            parameters=[{
                "input_topic": motor_topic,
                "odom_topic": raw_odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "wheel_radius": wheel_radius,
                "track_width": track_width,
                "left_wheel_direction": left_direction,
                "right_wheel_direction": right_direction,
                "use_encoder_revolutions": ParameterValue(
                    use_revolutions, value_type=bool
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "imu_parent_frame": LaunchConfiguration("imu_parent_frame"),
                "imu_frame": LaunchConfiguration("imu_frame"),
                "imu_x": LaunchConfiguration("imu_x"),
                "imu_y": LaunchConfiguration("imu_y"),
                "imu_z": LaunchConfiguration("imu_z"),
                "imu_roll": LaunchConfiguration("imu_roll"),
                "imu_pitch": LaunchConfiguration("imu_pitch"),
                "imu_yaw": LaunchConfiguration("imu_yaw"),
                "filtered_odom_topic": filtered_odom_topic,
                "reset_topic": LaunchConfiguration("reset_topic"),
                "reset_service": LaunchConfiguration("reset_service"),
                "reset_position_service": LaunchConfiguration(
                    "reset_position_service"
                ),
                "reset_orientation_service": LaunchConfiguration(
                    "reset_orientation_service"
                ),
                "set_pose_service": LaunchConfiguration("set_pose_service"),
                "reset_frame": LaunchConfiguration("reset_frame"),
                "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
            }.items(),
        ),
    ])
