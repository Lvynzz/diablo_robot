from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "diablo_full_body", package_name="diablo_full_body_moveit_config"
        )
        .robot_description()
        .robot_description_semantic(file_path="config/diablo_full_body.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(moveit_config.package_path / "config" / "moveit.rviz"),
        description="RViz configuration file",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="diablo_full_body_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([rviz_config, rviz])
