from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    group_name = DeclareLaunchArgument(
        "group_name", default_value="left_manipulator",
        description="MoveIt arm group used for position IK",
    )
    ik_link_name = DeclareLaunchArgument(
        "ik_link_name", default_value="left_arm_tip_link",
        description="IK tip link",
    )
    execute = DeclareLaunchArgument(
        "execute", default_value="false",
        description="false=plan only, true=execute in the simulated controller",
    )
    target_topic = DeclareLaunchArgument(
        "target_topic", default_value="/diablo/ik_target",
        description="PoseStamped target topic",
    )

    bridge = Node(
        package="diablo_moveit_bridge",
        executable="ik_moveit_bridge",
        name="diablo_full_body_ik_bridge",
        output="screen",
        parameters=[
            {
                "group_name": LaunchConfiguration("group_name"),
                "ik_link_name": LaunchConfiguration("ik_link_name"),
                "target_topic": LaunchConfiguration("target_topic"),
                "planning_frame": "torso_link",
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription([group_name, ik_link_name, execute, target_topic, bridge])
