import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _default_map_file(bringup_share):
    """Use the web-selected bringup map, then the first available map."""
    candidates = [Path(bringup_share) / "map"]
    for workspace_root in Path(bringup_share).parents:
        candidates.append(workspace_root / "src" / "diablo_bringup" / "map")
    # Do not return the first YAML before all candidates have been checked.
    # The install space can contain a stale selection marker while the actual
    # selected map is still only present in the source workspace.  Returning
    # ``empty.yaml`` here made Nav2 replace the HMI preview with a blank map.
    fallback = None
    for directory in candidates:
        marker = directory / ".selected_localization_map.txt"
        try:
            selected = marker.read_text(encoding="utf-8").strip()
        except OSError:
            selected = ""
        if selected:
            selected_path = Path(selected).expanduser()
            if selected_path.suffix.lower() == ".pgm":
                selected_path = selected_path.with_suffix(".yaml")
            # The install-space map files may be symlinks into the source
            # checkout.  Resolve the marker by basename inside each candidate
            # directory; resolving the file first makes the old parent check
            # reject valid install-space symlinks and fall back to empty.yaml.
            selected_path = directory / selected_path.name
            try:
                if selected_path.is_file():
                    return str(selected_path)
            except OSError:
                pass
        try:
            yaml_files = sorted(directory.glob("*.yaml"))
        except OSError:
            yaml_files = []
        if yaml_files and fallback is None:
            fallback = str(yaml_files[0])
    if fallback:
        return fallback
    return str(Path(bringup_share) / "map" / "empty.yaml")


def generate_launch_description():
    share = get_package_share_directory("diablo_web_interface")
    bringup_share = get_package_share_directory("diablo_bringup")
    default_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = _default_map_file(bringup_share)

    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_wheel_odom = LaunchConfiguration("enable_wheel_odom")
    enable_mux = LaunchConfiguration("enable_mux")
    mux_default_mode = LaunchConfiguration("mux_default_mode")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    control_mode_topic = LaunchConfiguration("control_mode_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    autostart_navigation = LaunchConfiguration("autostart_navigation")

    def bool_value(value):
        return ParameterValue(value, value_type=bool)

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "robot_base_frame": base_frame,
            "base_frame_id": base_frame,
            "odom_topic": odom_topic,
            "odom_frame_id": odom_frame,
            "topic": scan_topic,
            "scan_topic": scan_topic,
            # Navigation must wait for the web Set Initial Pose action;
            # do not silently initialize AMCL at (0, 0, 0) from YAML.
            "set_initial_pose": set_initial_pose,
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "enable_wheel_odom",
            default_value="false",
            description=(
                "Start legacy standalone wheel odometry. Keep false when "
                "full_body_hardware.launch.py provides local odometry."
            ),
        ),
        DeclareLaunchArgument("enable_mux", default_value="true"),
        DeclareLaunchArgument("mux_default_mode", default_value="auto"),
        DeclareLaunchArgument(
            "manual_cmd_topic", default_value="/diablo/MotionCmd/manual"
        ),
        DeclareLaunchArgument(
            "control_mode_topic", default_value="/diablo/control_mode"
        ),
        DeclareLaunchArgument("nav_cmd_topic", default_value="/cmd_vel_smoothed"),
        DeclareLaunchArgument("motor_topic", default_value="/diablo/sensor/Motors"),
        DeclareLaunchArgument("odom_topic", default_value="/diablo/odometry"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="diablo_base_link"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="false",
            description="Wait for /initialpose before AMCL publishes map->odom.",
        ),
        DeclareLaunchArgument(
            "autostart_navigation",
            default_value="false",
            description=(
                "Start Nav2 controller/planner/BT immediately. The web HMI "
                "keeps this false until AMCL publishes map->odom."
            ),
        ),
        DeclareLaunchArgument("wheel_radius", default_value="0.093"),
        DeclareLaunchArgument("track_width", default_value="0.475"),
        DeclareLaunchArgument("left_wheel_direction", default_value="1.0"),
        DeclareLaunchArgument("right_wheel_direction", default_value="1.0"),
        DeclareLaunchArgument("use_encoder_revolutions", default_value="true"),

        Node(
            package="diablo_web_interface",
            executable="wheel_odom",
            name="diablo_wheel_odom",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("motor_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "track_width": LaunchConfiguration("track_width"),
                "left_wheel_direction": LaunchConfiguration("left_wheel_direction"),
                "right_wheel_direction": LaunchConfiguration("right_wheel_direction"),
                "use_encoder_revolutions": bool_value(
                    LaunchConfiguration("use_encoder_revolutions")
                ),
            }],
            condition=IfCondition(enable_wheel_odom),
        ),
        Node(
            package="diablo_web_interface",
            executable="motion_cmd_bridge",
            name="diablo_motion_cmd_bridge",
            output="screen",
            parameters=[{
                "input_topic": nav_cmd_topic,
                "output_topic": "/diablo/MotionCmd/nav",
                "use_sim_time": bool_value(use_sim_time),
            }],
        ),
        Node(
            package="diablo_web_interface",
            executable="motion_cmd_mux",
            name="diablo_motion_cmd_mux",
            output="screen",
            parameters=[{
                "manual_topic": manual_cmd_topic,
                "nav_topic": "/diablo/MotionCmd/nav",
                "output_topic": "/diablo/MotionCmd",
                "control_mode_topic": control_mode_topic,
                "default_mode": mux_default_mode,
            }],
            condition=IfCondition(enable_mux),
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                configured_params,
                {
                    "use_sim_time": bool_value(use_sim_time),
                    "yaml_filename": ParameterValue(map_file, value_type=str),
                },
            ],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                configured_params,
                {
                    "use_sim_time": bool_value(use_sim_time),
                    "set_initial_pose": bool_value(set_initial_pose),
                },
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params, {"use_sim_time": bool_value(use_sim_time)}],
            remappings=[
                ("cmd_vel", "/cmd_vel_nav"),
                ("cmd_vel_smoothed", nav_cmd_topic),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                configured_params,
                {
                    "use_sim_time": bool_value(use_sim_time),
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                configured_params,
                {
                    "use_sim_time": bool_value(use_sim_time),
                    # Planner/controller costmaps require map->odom.  The web
                    # node requests STARTUP after Set Initial Pose so this
                    # manager cannot block waiting for a transform during
                    # Navigation launch.
                    "autostart": bool_value(autostart_navigation),
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ],
                },
            ],
        ),
    ])
