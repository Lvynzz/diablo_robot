from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "diablo_full_body", package_name="diablo_full_body_moveit_config"
        )
        .robot_description()
        .to_moveit_configs()
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Start RViz2"
    )
    package_share = FindPackageShare("diablo_full_body_moveit_config")

    robot_controllers = PathJoinSubstitution([
        package_share, "config", "ros2_controllers.yaml"
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="diablo_full_body_robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_controllers, moveit_config.robot_description],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diablo_base_controller", "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller", "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller", "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[base_controller, left_arm_controller, right_arm_controller],
        )
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package_share, "launch", "static_virtual_joint_tfs.launch.py"
        ]))
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package_share, "launch", "move_group.launch.py"
        ]))
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package_share, "launch", "moveit_rviz.launch.py"
        ])),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_rviz,
        static_tf,
        robot_state_publisher,
        ros2_control,
        joint_state_broadcaster,
        start_controllers,
        move_group,
        rviz,
    ])
