import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('diablo_bringup')
    xacro_file = os.path.join(pkg, 'urdf', 'two_joint.urdf.xacro')

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
            'update_rate': 100,
        }],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    return LaunchDescription([
        robot_state_publisher, controller_manager, spawn_jsb,
    ])
