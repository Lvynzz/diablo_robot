from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('diablo_bringup')

    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')

    declare_port = DeclareLaunchArgument(
        'port_name', default_value='/dev/ttyUSB0',
        description='Serial port U2D2 tempat motor terhubung')
    declare_baud = DeclareLaunchArgument(
        'baud_rate', default_value='1000000',
        description='Baud rate komunikasi ke Dynamixel')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'joint23.urdf.xacro'])

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' port_name:=', port_name,
        ' baud_rate:=', baud_rate,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'joint23_controllers.yaml'])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint23_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint23_trajectory_controller'],
        output='screen',
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint23_trajectory_controller_spawner,
    ])
