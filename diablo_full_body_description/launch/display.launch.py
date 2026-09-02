import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory("diablo_full_body_description")
    xacro_file = os.path.join(
        share, "description", "urdf", "diablo_full_body.urdf.xacro"
    )
    urdf = xacro.process_file(xacro_file).toxml()
    urdf_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="diablo_full_body_", suffix=".urdf", delete=False
    )
    urdf_file.write(urdf)
    urdf_file.close()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="diablo_full_body_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="diablo_full_body_joint_state_publisher_gui",
            output="screen",
            arguments=[urdf_file.name],
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="diablo_full_body_joint_state_publisher",
            output="screen",
            arguments=[urdf_file.name],
            condition=IfCondition(LaunchConfiguration("use_gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="diablo_full_body_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
    ])
