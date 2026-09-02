from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('diablo_bringup')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'joint123.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution(
        [pkg_share, 'config', 'joint123_controllers.yaml'])

    # Port dan baudrate memakai default di joint123.urdf.xacro.
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint123_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint123_trajectory_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint123_trajectory_controller_spawner,
    ])
