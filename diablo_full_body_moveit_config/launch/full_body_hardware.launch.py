import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_setup(context):
    description_share = get_package_share_directory("diablo_full_body_description")
    moveit_share = get_package_share_directory("diablo_full_body_moveit_config")
    xacro_file = os.path.join(
        description_share, "description", "urdf", "diablo_full_body.urdf.xacro"
    )
    controllers_file = os.path.join(moveit_share, "config", "ros2_controllers.yaml")

    mappings = {
        "use_mock_hardware": LaunchConfiguration("use_mock_hardware").perform(context),
        "enable_base_hardware": LaunchConfiguration("enable_base_hardware").perform(context),
        "enable_arm_hardware": LaunchConfiguration("enable_arm_hardware").perform(context),
        "upper_only": LaunchConfiguration("upper_only").perform(context),
        "arm_port_name": LaunchConfiguration("arm_port_name").perform(context),
        "hand_port_name": LaunchConfiguration("hand_port_name").perform(context),
        "baud_rate": LaunchConfiguration("baud_rate").perform(context),
        "wheel_radius": LaunchConfiguration("wheel_radius").perform(context),
        "track_width": LaunchConfiguration("track_width").perform(context),
        "left_feedback_sign": LaunchConfiguration("left_feedback_sign").perform(context),
        "right_feedback_sign": LaunchConfiguration("right_feedback_sign").perform(context),
        "use_ekf": LaunchConfiguration("use_ekf").perform(context),
        "use_local_odom": LaunchConfiguration("use_local_odom").perform(context),
    }
    robot_description = xacro.process_file(xacro_file, mappings=mappings).toxml()

    use_ekf_active = (
        mappings["use_ekf"].lower() == "true"
        and mappings["enable_base_hardware"].lower() == "true"
        and LaunchConfiguration("start_base_controller").perform(context).lower() == "true"
        and mappings["upper_only"].lower() != "true"
    )
    use_local_odom_active = (
        mappings["use_local_odom"].lower() == "true"
        and not use_ekf_active
        and mappings["enable_base_hardware"].lower() == "true"
        and LaunchConfiguration("start_base_controller").perform(context).lower() == "true"
        and mappings["upper_only"].lower() != "true"
    )

    controller_parameters = [controllers_file]
    localization_share = None
    if use_ekf_active:
        localization_share = get_package_share_directory("diablo_localization")
        ekf_controllers_file = os.path.join(
            moveit_share, "config", "ros2_controllers_ekf.yaml"
        )
        ekf_params_file = LaunchConfiguration("ekf_params_file").perform(context)
        if not ekf_params_file:
            ekf_params_file = os.path.join(
                localization_share, "config", "diablo_ekf.yaml"
            )
        # The second file overrides only enable_odom_tf.  This prevents the
        # raw diff-drive controller and EKF from publishing the same TF.
        controller_parameters.append(ekf_controllers_file)
    elif use_local_odom_active:
        localization_share = get_package_share_directory("diablo_localization")
        local_odom_controllers_file = os.path.join(
            moveit_share, "config", "ros2_controllers_local_odom.yaml"
        )
        # The resettable local odometry node owns the same TF in raw-odom
        # mode, so disable the diff-drive controller's duplicate TF.
        controller_parameters.append(local_odom_controllers_file)
    controller_parameters.append({"robot_description": robot_description})

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="diablo_full_body_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=controller_parameters,
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    controller_spawners = []
    if (
        LaunchConfiguration("start_arm_controllers").perform(context).lower() == "true"
        and mappings["enable_arm_hardware"].lower() == "true"
    ):
        controller_spawners.extend([left_arm_controller, right_arm_controller])

    if (
        LaunchConfiguration("start_base_controller").perform(context).lower() == "true"
        and mappings["enable_base_hardware"].lower() == "true"
        and mappings["upper_only"].lower() != "true"
    ):
        controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diablo_base_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        )

    actions = [
        robot_state_publisher,
        ros2_control,
        joint_state_broadcaster,
    ]
    if use_ekf_active:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(localization_share, "launch", "localization.launch.py")
                ),
                launch_arguments={
                    "params_file": ekf_params_file,
                    "imu_parent_frame": LaunchConfiguration("imu_parent_frame"),
                    "imu_frame": LaunchConfiguration("imu_frame"),
                    "imu_x": LaunchConfiguration("imu_x"),
                    "imu_y": LaunchConfiguration("imu_y"),
                    "imu_z": LaunchConfiguration("imu_z"),
                    "imu_roll": LaunchConfiguration("imu_roll"),
                    "imu_pitch": LaunchConfiguration("imu_pitch"),
                    "imu_yaw": LaunchConfiguration("imu_yaw"),
                    "reset_topic": LaunchConfiguration("reset_topic"),
                    "reset_service": LaunchConfiguration("reset_service"),
                    "set_pose_service": LaunchConfiguration("set_pose_service"),
                    "reset_frame": LaunchConfiguration("reset_frame"),
                    "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                }.items(),
            )
        )
    if use_local_odom_active:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(localization_share, "launch", "local_odom.launch.py")
                ),
                launch_arguments={
                    "input_odom_topic": "/diablo_base_controller/odom",
                    "output_odom_topic": "/diablo/odometry",
                    "odom_frame": "odom",
                    "base_frame": "diablo_base_link",
                    "reset_topic": LaunchConfiguration("reset_topic"),
                    "reset_service": LaunchConfiguration("reset_service"),
                    "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                    "publish_tf": "true",
                    "reset_on_start": LaunchConfiguration("reset_on_start"),
                }.items(),
            )
        )
    if controller_spawners:
        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=controller_spawners,
                )
            )
        )

    if LaunchConfiguration("start_move_group").perform(context).lower() == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "move_group.launch.py"])
                )
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "moveit_rviz.launch.py"])
                ),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock_components instead of the real hardware plugins",
        ),
        DeclareLaunchArgument(
            "upper_only",
            default_value="false",
            description="Build only the upper-body part of the existing URDF",
        ),
        DeclareLaunchArgument(
            "enable_base_hardware",
            default_value="true",
            description="Include the DiabloSystemHardware wheel adapter",
        ),
        DeclareLaunchArgument(
            "enable_arm_hardware",
            default_value="true",
            description="Include the two Dynamixel upper-body systems",
        ),
        DeclareLaunchArgument(
            "arm_port_name",
            default_value="/dev/u2d2_arm",
            description="U2D2-A port for arm Dynamixels",
        ),
        DeclareLaunchArgument(
            "hand_port_name",
            default_value="/dev/u2d2_hand",
            description="U2D2-B port for Seed Robotics hand Dynamixels",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("wheel_radius", default_value="0.105"),
        DeclareLaunchArgument("track_width", default_value="0.3751"),
        DeclareLaunchArgument(
            "left_feedback_sign",
            default_value="1.0",
            description="Sign applied to left wheel feedback before odometry",
        ),
        DeclareLaunchArgument(
            "right_feedback_sign",
            default_value="1.0",
            description="Sign applied to right wheel feedback before odometry",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description=(
                "Start the optional wheel/IMU robot_localization estimator "
                "instead of resettable local wheel odometry"
            ),
        ),
        DeclareLaunchArgument(
            "use_local_odom",
            default_value="true",
            description=(
                "Publish resettable local wheel odometry on /diablo/odometry "
                "and own odom -> diablo_base_link TF"
            ),
        ),
        DeclareLaunchArgument(
            "reset_on_start",
            default_value="false",
            description="Use the first raw odometry sample as local origin",
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value="",
            description="Optional robot_localization parameter file",
        ),
        DeclareLaunchArgument(
            "imu_parent_frame", default_value="diablo_base_link"
        ),
        DeclareLaunchArgument("imu_frame", default_value="diablo_robot"),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("reset_topic", default_value="/diablo/reset_pose"),
        DeclareLaunchArgument("reset_service", default_value="/diablo/reset_odom"),
        DeclareLaunchArgument(
            "set_pose_service", default_value="/set_pose"
        ),
        DeclareLaunchArgument("reset_frame", default_value="odom"),
        DeclareLaunchArgument(
            "stop_cmd_topic",
            default_value="/diablo_base_controller/cmd_vel_unstamped",
        ),
        DeclareLaunchArgument(
            "start_base_controller",
            default_value="true",
            description="Spawn diff_drive_controller (disable for upper-only tests)",
        ),
        DeclareLaunchArgument(
            "start_arm_controllers",
            default_value="true",
            description="Spawn left/right joint trajectory controllers",
        ),
        DeclareLaunchArgument(
            "start_move_group",
            default_value="false",
            description="Also start MoveIt move_group",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
