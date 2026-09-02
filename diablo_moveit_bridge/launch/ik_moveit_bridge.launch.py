from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("diablo_moveit_bridge"),
        "config",
        "ik_moveit_bridge.yaml",
    ])

    group_name = DeclareLaunchArgument(
        "group_name",
        default_value="left_manipulator",
        description="MoveIt planning group",
    )
    ik_link_name = DeclareLaunchArgument(
        "ik_link_name",
        default_value="left_arm_tip_link",
        description="MoveIt IK tip link",
    )
    execute = DeclareLaunchArgument(
        "execute",
        default_value="false",
        description="false=plan only; true=allow MoveIt to execute",
    )
    fallback_enabled = DeclareLaunchArgument(
        "fallback_enabled",
        default_value="true",
        description="search nearby reachable XYZ when the requested target has no IK solution",
    )
    avoid_collisions = DeclareLaunchArgument(
        "avoid_collisions",
        default_value="true",
        description="ask MoveIt IK to reject self/world-colliding solutions",
    )
    pointing_enabled = DeclareLaunchArgument(
        "pointing_enabled",
        default_value="true",
        description="project far targets onto a reachable straight-pointing pose",
    )
    pointing_lock_elbow = DeclareLaunchArgument(
        "pointing_lock_elbow",
        default_value="false",
        description="hard-constrain the elbow near the straight pointing angle",
    )

    node = Node(
        package="diablo_moveit_bridge",
        executable="ik_moveit_bridge",
        name="ik_moveit_bridge",
        output="screen",
        parameters=[
            config_file,
            {
                "group_name": LaunchConfiguration("group_name"),
                "ik_link_name": LaunchConfiguration("ik_link_name"),
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "fallback_enabled": ParameterValue(
                    LaunchConfiguration("fallback_enabled"), value_type=bool
                ),
                "avoid_collisions": ParameterValue(
                    LaunchConfiguration("avoid_collisions"), value_type=bool
                ),
                "pointing_enabled": ParameterValue(
                    LaunchConfiguration("pointing_enabled"), value_type=bool
                ),
                "pointing_lock_elbow": ParameterValue(
                    LaunchConfiguration("pointing_lock_elbow"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        group_name,
        ik_link_name,
        execute,
        fallback_enabled,
        avoid_collisions,
        pointing_enabled,
        pointing_lock_elbow,
        node,
    ])
