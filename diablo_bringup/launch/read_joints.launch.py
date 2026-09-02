import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('diablo_bringup')
    xacro_file = os.path.join(pkg, 'urdf', 'six_joint.urdf.xacro')

    update_rate_arg = DeclareLaunchArgument('update_rate', default_value='30')

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, {
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'update_rate': LaunchConfiguration('update_rate'),
        }],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    return LaunchDescription([
        update_rate_arg, robot_state_publisher, controller_manager, spawn_jsb,
    ])
