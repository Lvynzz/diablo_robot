from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="diablo_world_to_base",
            output="screen",
            arguments=[
                "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
                "world", "diablo_base_link",
            ],
        )
    ])
