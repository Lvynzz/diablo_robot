#!/usr/bin/env python3
"""ROS 2 state, teleoperation and Nav2 integration for the Diablo web UI."""

from collections import OrderedDict
import copy
import math
from pathlib import Path
import re
import shlex
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from motion_msgs.msg import LegMotors, MotionCtrl, RobotStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

from .hardware_manager import HardwareManager


MAX_ECHO_DEPTH = 5
MAX_ECHO_ITEMS = 80
MAX_LIDAR_POINTS = 720
MAX_MAP_CELLS = 250_000
# Costmaps are visual diagnostics only; Nav2 itself consumes the native ROS
# grids.  A smaller web representation keeps JSON packets and browser memory
# bounded without changing navigation behaviour.
MAX_COSTMAP_CELLS = 30_000
# Costmaps are refreshed much more frequently than the operator can inspect
# them.  Do not parse and serialize a rolling grid for every DDS sample.  This
# also prevents a high-rate local costmap from monopolising the web executor.
LOCAL_COSTMAP_MIN_PERIOD = 0.20
GLOBAL_COSTMAP_MIN_PERIOD = 0.50
MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
MAP_SELECTION_FILENAME = ".selected_localization_map.txt"


def _map_stem(value):
    """Return a safe map stem from a user/API value, or ``None``."""
    requested = str(value or "").strip()
    for suffix in (".pgm", ".yaml"):
        if requested.lower().endswith(suffix):
            requested = requested[: -len(suffix)]
            break
    return requested if MAP_NAME_PATTERN.fullmatch(requested) else None


def _read_pgm(path):
    """Read a PGM image without requiring Pillow (map preview helper)."""
    raw = Path(path).read_bytes()
    tokens = []
    index = 0
    length = len(raw)
    while index < length and len(tokens) < 4:
        while index < length and raw[index] in b" \t\r\n":
            index += 1
        if index < length and raw[index] == ord("#"):
            newline = raw.find(b"\n", index)
            index = length if newline < 0 else newline + 1
            continue
        start = index
        while index < length and raw[index] not in b" \t\r\n#":
            index += 1
        if index > start:
            tokens.append(raw[start:index].decode("ascii"))
        else:
            index += 1
    if len(tokens) != 4 or tokens[0] not in ("P2", "P5"):
        raise ValueError(f"Unsupported PGM format in {Path(path).name}")
    width, height, maximum = (int(tokens[1]), int(tokens[2]), int(tokens[3]))
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
        raise ValueError(f"Invalid PGM dimensions in {Path(path).name}")
    if index < length and raw[index] in b" \t\r\n":
        line_break = raw[index] == ord("\r")
        index += 1
        if line_break and index < length and raw[index] == ord("\n"):
            index += 1
    count = width * height
    if tokens[0] == "P2":
        values = []
        for token in raw[index:].split():
            if token.startswith(b"#"):
                continue
            values.append(int(token))
            if len(values) >= count:
                break
    else:
        bytes_per_value = 1 if maximum < 256 else 2
        payload = raw[index : index + count * bytes_per_value]
        if len(payload) < count * bytes_per_value:
            raise ValueError(f"Truncated PGM data in {Path(path).name}")
        if bytes_per_value == 1:
            values = list(payload[:count])
        else:
            values = [payload[i] * 256 + payload[i + 1] for i in range(0, count * 2, 2)]
    if len(values) != count:
        raise ValueError(f"Truncated PGM data in {Path(path).name}")
    return width, height, maximum, values


def _read_map_yaml(path):
    """Read the small scalar subset used by nav2 map YAML files."""
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            try:
                values[key.strip()] = [float(item.strip()) for item in value[1:-1].split(",")]
            except ValueError:
                continue
        elif value:
            try:
                values[key.strip()] = float(value)
            except ValueError:
                values[key.strip()] = value.strip("'\"")
    return values

JOINT_DEFINITIONS = {
    1: {"label": "RIGHT SHOULDER PITCH", "side": "right", "name": "upper_right_shoulder_pitch_joint", "min": -math.pi, "max": math.pi},
    2: {"label": "RIGHT SHOULDER ROLL", "side": "right", "name": "upper_right_shoulder_roll_joint", "min": 0.0, "max": 2.20},
    3: {"label": "RIGHT ELBOW", "side": "right", "name": "upper_right_elbow_joint", "min": -0.0872, "max": 2.35},
    4: {"label": "RIGHT WRIST", "side": "right", "name": "upper_right_wrist_joint", "min": -1.57, "max": 1.57},
    5: {"label": "RIGHT THUMB BASE", "side": "right", "name": "upper_right_thumb_base", "min": -0.785, "max": 0.785},
    6: {"label": "LEFT SHOULDER PITCH", "side": "left", "name": "upper_left_shoulder_pitch_joint", "min": -math.pi, "max": math.pi},
    7: {"label": "LEFT SHOULDER ROLL", "side": "left", "name": "upper_left_shoulder_roll_joint", "min": 0.0, "max": 2.20},
    8: {"label": "LEFT ELBOW", "side": "left", "name": "upper_left_elbow_joint", "min": -0.0872, "max": 2.35},
    9: {"label": "LEFT WRIST", "side": "left", "name": "upper_left_wrist_joint", "min": -1.57, "max": 1.57},
    10: {"label": "LEFT THUMB BASE", "side": "left", "name": "upper_left_thumb_base", "min": -0.785, "max": 0.785},
}


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def _bounded_value(value, depth=0):
    """Convert a ROS value to JSON-safe data with predictable size limits."""
    if depth > MAX_ECHO_DEPTH:
        return "<max-depth>"
    if isinstance(value, (OrderedDict, dict)):
        items = list(value.items())
        bounded = {
            str(key): _bounded_value(item, depth + 1)
            for key, item in items[:MAX_ECHO_ITEMS]
        }
        if len(items) > MAX_ECHO_ITEMS:
            bounded["<truncated>"] = len(items) - MAX_ECHO_ITEMS
        return bounded
    if isinstance(value, (list, tuple)):
        bounded = [_bounded_value(item, depth + 1) for item in value[:MAX_ECHO_ITEMS]]
        if len(value) > MAX_ECHO_ITEMS:
            bounded.append(f"<truncated {len(value) - MAX_ECHO_ITEMS} items>")
        return bounded
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return value
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def ros_value_to_bounded_data(message):
    """Serialize an arbitrary ROS message for the topic echo cards."""
    return _bounded_value(message_to_ordereddict(message))


class DiabloWebNode(Node):
    """Thread-safe ROS node used by the FastAPI application."""

    def __init__(self):
        super().__init__("diablo_web_node")

        self.declare_parameter("manual_cmd_topic", "/diablo/MotionCmd/manual")
        self.declare_parameter("control_mode_topic", "/diablo/control_mode")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "diablo_base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        # Keep the calibrated laser pose available to the web bridge even
        # during the short DDS discovery window before /tf_static is latched.
        # These values are also the same defaults used by web_interface.launch.
        self.declare_parameter("lidar_x", 0.0)
        self.declare_parameter("lidar_y", 0.08)
        self.declare_parameter("lidar_z", 0.17)
        self.declare_parameter("lidar_yaw", 3.141592653589793)
        self.declare_parameter("max_forward_command", 1.0)
        self.declare_parameter("max_turn_command", 1.0)
        self.declare_parameter("max_roll_command", 0.2)
        self.declare_parameter("default_up", 1.0)
        self.declare_parameter("reset_encoder_service", "/diablo/reset_encoder")
        self.declare_parameter("lidar_start_service", "/start_motor")
        self.declare_parameter("lidar_start_service_type", "empty")
        self.declare_parameter("lidar_stop_service", "/stop_motor")
        self.declare_parameter("lidar_stop_service_type", "empty")
        self.declare_parameter(
            "diablo_start_command",
            "ros2 launch diablo_localization ekf_hardware.launch.py "
            "controller_port:=/dev/diablo_controller",
        )
        self.declare_parameter(
            "lidar_start_command",
            "ros2 launch sllidar_ros2 sllidar_a2m7_launch.py serial_port:=/dev/rplidar frame_id:=laser",
        )
        self.declare_parameter(
            "dynamixel_start_command",
            "ros2 launch diablo_full_body_moveit_config full_body_hardware.launch.py "
            "use_mock_hardware:=false upper_only:=true "
            "enable_arm_hardware:=true enable_hand_hardware:=false enable_base_hardware:=false "
            "arm_port_name:=/dev/u2d2_arm hand_port_name:=/dev/u2d2_hand baud_rate:=1000000 "
            "start_arm_controllers:=true start_base_controller:=false use_ekf:=true "
            "use_local_odom:=false start_move_group:=false",
        )
        self.declare_parameter("hardware_log_directory", "/tmp")
        self.declare_parameter("hardware_feedback_timeout", 15.0)
        self.declare_parameter(
            "localization_start_command",
            "ros2 launch diablo_web_interface localization.launch.py",
        )
        self.declare_parameter(
            "navigation_start_command",
            "ros2 launch diablo_web_interface navigation.launch.py "
            "enable_mux:=false enable_wheel_odom:=false "
            "autostart_navigation:=false",
        )
        self.declare_parameter(
            "mapping_start_command",
            "ros2 launch diablo_web_interface mapping.launch.py enable_wheel_odom:=false scan_topic:=/scan",
        )
        self.declare_parameter("maps_dir", "")
        self.declare_parameter(
            "left_arm_trajectory_topic", "/left_arm_controller/joint_trajectory"
        )
        self.declare_parameter(
            "right_arm_trajectory_topic", "/right_arm_controller/joint_trajectory"
        )

        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.control_mode_topic = str(self.get_parameter("control_mode_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.lidar_x = float(self.get_parameter("lidar_x").value)
        self.lidar_y = float(self.get_parameter("lidar_y").value)
        self.lidar_z = float(self.get_parameter("lidar_z").value)
        self.lidar_yaw = float(self.get_parameter("lidar_yaw").value)
        if not all(math.isfinite(value) for value in (
            self.lidar_x, self.lidar_y, self.lidar_z, self.lidar_yaw
        )):
            raise ValueError("lidar pose parameters must be finite")
        self.max_forward = abs(float(self.get_parameter("max_forward_command").value))
        self.max_turn = abs(float(self.get_parameter("max_turn_command").value))
        self.max_roll = abs(float(self.get_parameter("max_roll_command").value))
        self.default_up = float(self.get_parameter("default_up").value)
        self.reset_encoder_service = str(
            self.get_parameter("reset_encoder_service").value
        ).strip()
        self.lidar_start_service = str(
            self.get_parameter("lidar_start_service").value
        ).strip()
        self.lidar_start_service_type = str(
            self.get_parameter("lidar_start_service_type").value
        ).strip().lower()
        if self.lidar_start_service_type not in {"empty", "trigger"}:
            self.get_logger().warning(
                f"Unknown lidar_start_service_type '{self.lidar_start_service_type}'; "
                "using 'empty'"
            )
            self.lidar_start_service_type = "empty"
        self.lidar_stop_service = str(
            self.get_parameter("lidar_stop_service").value
        ).strip()
        self.lidar_stop_service_type = str(
            self.get_parameter("lidar_stop_service_type").value
        ).strip().lower()
        if self.lidar_stop_service_type not in {"empty", "trigger"}:
            self.get_logger().warning(
                f"Unknown lidar_stop_service_type '{self.lidar_stop_service_type}'; "
                "using 'empty'"
            )
            self.lidar_stop_service_type = "empty"
        self.diablo_start_command = str(
            self.get_parameter("diablo_start_command").value
        ).strip()
        self.lidar_start_command = str(
            self.get_parameter("lidar_start_command").value
        ).strip()
        self.dynamixel_start_command = str(
            self.get_parameter("dynamixel_start_command").value
        ).strip()
        self.hardware_log_directory = str(
            self.get_parameter("hardware_log_directory").value
        ).strip()
        self.hardware_feedback_timeout = max(
            1.0, float(self.get_parameter("hardware_feedback_timeout").value)
        )
        self.localization_start_command = str(
            self.get_parameter("localization_start_command").value
        ).strip()
        self.navigation_start_command = str(
            self.get_parameter("navigation_start_command").value
        ).strip()
        self.mapping_start_command = str(
            self.get_parameter("mapping_start_command").value
        ).strip()
        self.left_arm_trajectory_topic = str(
            self.get_parameter("left_arm_trajectory_topic").value
        ).strip()
        self.right_arm_trajectory_topic = str(
            self.get_parameter("right_arm_trajectory_topic").value
        ).strip()
        configured_maps_dir = str(self.get_parameter("maps_dir").value).strip()
        if configured_maps_dir:
            self.maps_dir = Path(configured_maps_dir).expanduser()
        else:
            self.maps_dir = self._default_maps_dir()

        self._lock = threading.RLock()
        self._versions = {
            "map": 0,
            "local_costmap": 0,
            "global_costmap": 0,
            "path": 0,
            "scan": 0,
        }
        self._map = None
        self._local_costmap = None
        self._global_costmap = None
        self._path = None
        self._scan = None
        self._pose = None
        self._odom_pose = None
        self._last_amcl_pose_time = 0.0
        self._last_map_time = 0.0
        self._last_local_costmap_time = 0.0
        self._last_global_costmap_time = 0.0
        self._last_local_costmap_parse_time = 0.0
        self._last_global_costmap_parse_time = 0.0
        self._last_odom_time = 0.0
        self._last_scan_time = 0.0
        self._last_tf_time = 0.0
        self._last_footprint_time = 0.0
        self._footprint = None
        # These timestamps/values are diagnostic only.  Nav2 is the sole
        # publisher of the command chain; the web node merely observes every
        # hop so a blocked goal can explain exactly where velocity stopped.
        self._command_pipeline = {
            "cmd_vel_nav": {"topic": "/cmd_vel_nav", "last_time": 0.0, "linear": 0.0, "angular": 0.0},
            "cmd_vel_smoothed": {"topic": "/cmd_vel_smoothed", "last_time": 0.0, "linear": 0.0, "angular": 0.0},
            "motion_cmd_nav": {"topic": "/diablo/MotionCmd/nav", "last_time": 0.0, "linear": 0.0, "angular": 0.0},
            "motion_cmd_mux": {"topic": "/diablo/MotionCmd", "last_time": 0.0, "linear": 0.0, "angular": 0.0},
        }
        self._pending_initial_pose = None
        self._initial_pose_diagnostic = {
            "state": "idle",
            "message": "Belum ada initial pose",
            "attempts": 0,
            "age": None,
        }
        # Navigation's lifecycle manager is deliberately started only after
        # AMCL has produced map->odom.  Starting the planner/controller
        # costmaps before that transform exists makes planner_server block in
        # on_activate(), which leaves bt_navigator and velocity_smoother
        # inactive forever.  Keep the state here so the ROS timer can request
        # the lifecycle transition without blocking the HTTP thread.
        self._navigation_lifecycle_active = False
        self._navigation_lifecycle_state = "idle"
        self._navigation_lifecycle_start_future = None
        self._navigation_lifecycle_query_future = None
        self._navigation_lifecycle_last_request = 0.0
        self._navigation_lifecycle_last_query = 0.0
        # A direct TF observation complements tf2_ros.TransformListener.
        # Some robot images use a long-lived DDS participant where the
        # listener's buffer can miss the first dynamic transform; caching the
        # two frames needed by Nav2 lets the HMI keep rendering and gating on
        # the same TF that Nav2 is already receiving.
        self._dynamic_tf = {
            "map_odom": None,
            "odom_base": None,
        }
        self._wheel_trajectory = []
        self._telemetry = {
            "battery": None,
            "body_state": None,
            "imu": None,
            "motors": None,
        }
        self._joint_positions = {}
        self._control_mode = "manual"

        self._nav_goal_lock = threading.RLock()
        self._goal_sequence = 0
        self._current_goal_handle = None
        self._nav_goal_status = {
            "state": "idle",
            "message": "No navigation goal",
            "seq": 0,
            "distance_remaining": None,
        }

        self._manual_publisher = self.create_publisher(MotionCtrl, self.manual_cmd_topic, 10)
        self._control_mode_publisher = self.create_publisher(String, self.control_mode_topic, 10)
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._joint_publishers = {
            "left": self.create_publisher(
                JointTrajectory, self.left_arm_trajectory_topic, 10
            ),
            "right": self.create_publisher(
                JointTrajectory, self.right_arm_trajectory_topic, 10
            ),
        }
        self._reset_odom_client = self.create_client(Trigger, "/diablo/reset_odom")
        self._reset_position_client = self.create_client(Trigger, "/diablo/reset_position")
        self._reset_orientation_client = self.create_client(Trigger, "/diablo/reset_orientation")
        self._reset_encoder_client = (
            self.create_client(Trigger, self.reset_encoder_service)
            if self.reset_encoder_service
            else None
        )
        # Do not create both service types on the same ROS service name.  ROS 2
        # represents the request type in the DDS topic name, so doing that
        # causes an RCLError during startup as soon as a driver already owns
        # the service.  The bundled sllidar driver uses Empty; Trigger remains
        # available for drivers that expose a Trigger-based start service.
        self._lidar_start_client = None
        self._lidar_start_empty_client = None
        if self.lidar_start_service and not self.lidar_start_command:
            if self.lidar_start_service_type == "trigger":
                self._lidar_start_client = self.create_client(
                    Trigger, self.lidar_start_service
                )
            else:
                self._lidar_start_empty_client = self.create_client(
                    Empty, self.lidar_start_service
                )
        self._lidar_stop_client = None
        self._lidar_stop_empty_client = None
        if self.lidar_stop_service:
            if self.lidar_stop_service_type == "trigger":
                self._lidar_stop_client = self.create_client(
                    Trigger, self.lidar_stop_service
                )
            else:
                self._lidar_stop_empty_client = self.create_client(
                    Empty, self.lidar_stop_service
                )
        self._hardware = HardwareManager(
            self.get_logger(),
            diablo_command=self.diablo_start_command,
            lidar_command=self.lidar_start_command,
            dynamixel_command=self.dynamixel_start_command,
            lidar_topic=self.scan_topic,
            log_directory=self.hardware_log_directory,
            feedback_timeout=self.hardware_feedback_timeout,
        )

        # A fresh web bridge must begin in a known OFF state.  This cleanup
        # handles driver/Nav2/SLAM processes left by an earlier launch, while
        # preserving only the support nodes created by the new web launch.
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(f"Could not publish startup stop: {error}")
        try:
            # Ask a still-running scanner to stop its motor before the process
            # cleanup sends SIGTERM.  If the service is unavailable, the
            # process-group cleanup below remains the fallback.
            self.stop_lidar()
        except Exception as error:
            self.get_logger().warning(f"Could not request startup LiDAR stop: {error}")
        try:
            cleanup = self._hardware.startup_cleanup()
            if cleanup.get("requested"):
                self.get_logger().warning(
                    "Startup cleanup stopped stale robot processes before enabling the web UI"
                )
        except Exception as error:
            self.get_logger().error(f"Startup process cleanup failed: {error}")

        # Match Nav2/AMR QoS: /map and global costmap are latched reliable
        # grids, while the rolling local costmap and sensor streams are
        # best-effort volatile.  A sensor-data QoS on the global costmap can
        # miss its transient sample when navigation is enabled after the web
        # node has already subscribed.
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create TF before subscriptions so a latched costmap callback can
        # transform its odom-frame origin immediately after startup.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_grid_tf_warn = {}

        self._subscriptions = [
            self.create_subscription(
                OccupancyGrid, self.map_topic, self._map_callback, transient_qos
            ),
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self._local_costmap_callback,
                best_effort_qos,
            ),
            self.create_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._global_costmap_callback,
                transient_qos,
            ),
            # Keep a small copy of AMCL's dynamic transforms in addition to
            # tf2_ros' buffer.  The copy is used only as a startup/rendering
            # fallback; Nav2 remains the sole TF authority.
            self.create_subscription(
                TFMessage,
                "/tf",
                self._tf_message_callback,
                reliable_qos,
            ),
            self.create_subscription(
                PolygonStamped,
                "/local_costmap/published_footprint",
                self._footprint_callback,
                reliable_qos,
            ),
            self.create_subscription(
                Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data
            ),
            self.create_subscription(
                NavPath, "/plan", self._path_callback, qos_profile_sensor_data
            ),
            self.create_subscription(
                LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
            ),
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self._amcl_pose_callback,
                reliable_qos,
            ),
            self.create_subscription(
                BatteryState,
                "/diablo/sensor/Battery",
                self._battery_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                RobotStatus,
                "/diablo/sensor/Body_state",
                self._body_state_callback,
                10,
            ),
            self.create_subscription(
                Imu,
                "/diablo/sensor/Imu",
                self._imu_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LegMotors,
                "/diablo/sensor/Motors",
                self._motors_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                JointState,
                "/joint_states",
                self._joint_state_callback,
                qos_profile_sensor_data,
            ),
            # Observe each Nav2 command hop.  The topic names mirror
            # navigation.launch.py (controller -> smoother -> collision
            # monitor -> bridge -> mux).
            self.create_subscription(
                Twist, "/cmd_vel_nav", self._cmd_vel_nav_callback, 10
            ),
            self.create_subscription(
                Twist, "/cmd_vel_smoothed", self._cmd_vel_smoothed_callback, 10
            ),
            self.create_subscription(
                MotionCtrl,
                "/diablo/MotionCmd/nav",
                self._motion_cmd_nav_callback,
                10,
            ),
            self.create_subscription(
                MotionCtrl,
                "/diablo/MotionCmd",
                self._motion_cmd_mux_callback,
                10,
            ),
        ]

        self._tf_timer = self.create_timer(0.1, self._update_tf_pose)
        self._hardware_timer = self.create_timer(0.5, self._update_hardware_status)
        self._initial_pose_timer = self.create_timer(0.5, self._retry_initial_pose)
        self._navigation_lifecycle_timer = self.create_timer(
            0.5, self._navigation_lifecycle_tick
        )
        self._nav_to_pose_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._navigation_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self._navigation_lifecycle_active_client = self.create_client(
            Trigger,
            "/lifecycle_manager_navigation/is_active",
        )

        self.get_logger().info(
            f"Diablo web ROS node ready. Manual topic: {self.manual_cmd_topic}"
        )

    # -------------------- State callbacks --------------------

    def _map_callback(self, message):
        with self._lock:
            self._map = self._parse_grid(message)
            self._last_map_time = time.monotonic()
            self._versions["map"] += 1

    def _local_costmap_callback(self, message):
        received = time.monotonic()
        with self._lock:
            if (
                received - self._last_local_costmap_parse_time
                < LOCAL_COSTMAP_MIN_PERIOD
            ):
                return
            self._last_local_costmap_parse_time = received
        # Parsing is intentionally outside the state lock.  It can walk tens
        # of thousands of cells and must not pause pose/teleop snapshots.
        parsed = self._parse_grid(message, MAX_COSTMAP_CELLS)
        with self._lock:
            self._local_costmap = parsed
            self._last_local_costmap_time = received
            self._versions["local_costmap"] += 1

    def _global_costmap_callback(self, message):
        received = time.monotonic()
        with self._lock:
            if (
                received - self._last_global_costmap_parse_time
                < GLOBAL_COSTMAP_MIN_PERIOD
            ):
                return
            self._last_global_costmap_parse_time = received
        # Keep the ROS executor responsive while converting the grid to JSON.
        parsed = self._parse_grid(message, MAX_COSTMAP_CELLS)
        with self._lock:
            self._global_costmap = parsed
            self._last_global_costmap_time = received
            self._versions["global_costmap"] += 1

    def _tf_message_callback(self, message: TFMessage):
        """Cache the dynamic transforms required by AMCL and the costmaps.

        Nav2 publishes ``map -> odom`` from AMCL and the wheel odometry node
        publishes ``odom -> diablo_base_link``.  Keeping only these two links
        is enough to reconstruct the robot pose and transform the rolling
        local costmap when the Python tf2 listener has not populated its
        buffer yet.
        """
        received = time.monotonic()
        updates = {}
        for item in message.transforms:
            parent = str(item.header.frame_id or "").strip().lstrip("/")
            child = str(item.child_frame_id or "").strip().lstrip("/")
            if parent == self.map_frame and child == self.odom_frame:
                key = "map_odom"
            elif parent == self.odom_frame and child == self.base_frame:
                key = "odom_base"
            else:
                continue
            updates[key] = {
                "x": float(item.transform.translation.x),
                "y": float(item.transform.translation.y),
                "yaw": _yaw_from_quaternion(item.transform.rotation),
                "received": received,
            }
        if updates:
            with self._lock:
                self._dynamic_tf.update(updates)

    def _footprint_callback(self, message: PolygonStamped):
        """Keep Nav2's live footprint in map coordinates for the canvas."""
        source_frame = (message.header.frame_id or self.base_frame).strip().lstrip("/")
        points = []
        transform_ok = source_frame == self.map_frame
        transform = None
        if not transform_ok:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time()
                )
                transform_ok = True
            except Exception:
                transform_ok = False
        if transform_ok and transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tf_yaw = _yaw_from_quaternion(rotation)
            cosine = math.cos(tf_yaw)
            sine = math.sin(tf_yaw)
            for point in message.polygon.points:
                px = float(point.x)
                py = float(point.y)
                points.append({
                    "x": float(translation.x) + cosine * px - sine * py,
                    "y": float(translation.y) + sine * px + cosine * py,
                })
        else:
            points = [{"x": float(point.x), "y": float(point.y)} for point in message.polygon.points]
        with self._lock:
            self._footprint = {
                "frame_id": self.map_frame if transform_ok else source_frame,
                "source_frame_id": source_frame,
                "transform_ok": transform_ok,
                "points": points,
                "stamp": time.time(),
            }
            self._last_footprint_time = time.monotonic()

    def _odom_callback(self, message: Odometry):
        pose = self._pose_from_pose_message(message.pose.pose, "filtered_odom")
        with self._lock:
            self._odom_pose = pose
            self._last_odom_time = time.monotonic()
            if not self._wheel_trajectory:
                self._wheel_trajectory.append({"x": pose["x"], "y": pose["y"]})
            else:
                previous = self._wheel_trajectory[-1]
                if (
                    math.hypot(
                        pose["x"] - previous["x"], pose["y"] - previous["y"]
                    )
                    >= 0.002
                ):
                    self._wheel_trajectory.append(
                        {"x": pose["x"], "y": pose["y"]}
                    )
                    self._wheel_trajectory = self._wheel_trajectory[-600:]
            if self._pose is None or self._pose.get("source") in (
                "odom", "wheel_odom", "filtered_odom"
            ):
                self._pose = pose

    def _path_callback(self, message: NavPath):
        """Cache Nav2's global plan in the same frame as the map canvas.

        Nav2 normally publishes ``/plan`` in its configured global frame
        (``map``), but custom planner configurations may use ``odom``.  The
        browser only has one world-to-canvas transform, so convert the whole
        path once here and explicitly mark a path as unavailable when its TF
        cannot be resolved instead of drawing it at a misleading offset.
        """
        source_frame = (message.header.frame_id or self.map_frame).strip().lstrip("/")
        transform_ok = source_frame == self.map_frame
        transform = None
        if not transform_ok:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time()
                )
                transform_ok = True
            except Exception:
                transform = self._cached_transform(self.map_frame, source_frame)
                transform_ok = transform is not None

        if transform_ok and transform is not None:
            if hasattr(transform, "transform"):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                transform_x = float(translation.x)
                transform_y = float(translation.y)
                transform_yaw = _yaw_from_quaternion(rotation)
            else:
                # _cached_transform() returns a small dictionary when the
                # tf2 listener has not received the latest transform yet.
                transform_x = float(transform.get("x", 0.0))
                transform_y = float(transform.get("y", 0.0))
                transform_yaw = float(transform.get("yaw", 0.0))
            cosine = math.cos(transform_yaw)
            sine = math.sin(transform_yaw)
        else:
            transform_x = transform_y = transform_yaw = 0.0
            cosine = 1.0
            sine = 0.0

        points = []
        for item in message.poses:
            source_x = float(item.pose.position.x)
            source_y = float(item.pose.position.y)
            if transform_ok:
                x = transform_x + cosine * source_x - sine * source_y
                y = transform_y + sine * source_x + cosine * source_y
            else:
                x = source_x
                y = source_y
            points.append({"x": round(x, 4), "y": round(y, 4)})
        with self._lock:
            self._path = {
                "frame_id": self.map_frame if transform_ok else source_frame,
                "source_frame_id": source_frame,
                "transform_ok": transform_ok,
                "poses": points,
            }
            self._versions["path"] += 1

    def _scan_callback(self, message: LaserScan):
        self._hardware.mark_message("lidar")
        ranges = list(message.ranges)
        stride = max(1, math.ceil(len(ranges) / MAX_LIDAR_POINTS))
        sampled = ranges[::stride]
        clean_ranges = [
            round(float(value), 3) if math.isfinite(float(value)) else None
            for value in sampled
        ]
        # Use the launch calibration as a deterministic fallback.  Without
        # this, the browser silently draws the scan in the wrong direction if
        # a latched /tf_static sample arrives after the first LaserScan.
        sensor_x = self.lidar_x
        sensor_y = self.lidar_y
        sensor_theta = self.lidar_yaw
        try:
            scan_frame = message.header.frame_id or "laser"
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, scan_frame, Time()
            )
            sensor_x = float(transform.transform.translation.x)
            sensor_y = float(transform.transform.translation.y)
            sensor_theta = _yaw_from_quaternion(transform.transform.rotation)
        except Exception:
            # The calibrated fallback above remains valid for the configured
            # `laser` frame; subsequent scans still retry the lookup so custom
            # frame trees continue to work when they become available.
            pass
        with self._lock:
            self._scan = {
                "frame_id": message.header.frame_id,
                "sensor_x": sensor_x,
                "sensor_y": sensor_y,
                "sensor_theta": sensor_theta,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment) * stride,
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "ranges": clean_ranges,
            }
            self._last_scan_time = time.monotonic()
            self._versions["scan"] += 1

    def _cmd_vel_nav_callback(self, message: Twist):
        self._record_pipeline("cmd_vel_nav", float(message.linear.x), float(message.angular.z))

    def _cmd_vel_smoothed_callback(self, message: Twist):
        self._record_pipeline(
            "cmd_vel_smoothed", float(message.linear.x), float(message.angular.z)
        )

    def _motion_cmd_nav_callback(self, message: MotionCtrl):
        value = getattr(message, "value", None)
        self._record_pipeline(
            "motion_cmd_nav",
            float(getattr(value, "forward", 0.0)),
            float(getattr(value, "left", 0.0)),
        )

    def _motion_cmd_mux_callback(self, message: MotionCtrl):
        value = getattr(message, "value", None)
        self._record_pipeline(
            "motion_cmd_mux",
            float(getattr(value, "forward", 0.0)),
            float(getattr(value, "left", 0.0)),
        )

    def _record_pipeline(self, key, linear, angular):
        if not math.isfinite(linear):
            linear = 0.0
        if not math.isfinite(angular):
            angular = 0.0
        with self._lock:
            item = self._command_pipeline.get(key)
            if item is not None:
                item["last_time"] = time.monotonic()
                item["linear"] = linear
                item["angular"] = angular

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped):
        """Use AMCL's filtered map pose as the robot pose shown by the HMI."""
        pose = self._pose_from_pose_message(message.pose.pose, "amcl")
        with self._lock:
            self._pose = pose
            self._last_amcl_pose_time = time.monotonic()
            pending = self._pending_initial_pose
            if pending is not None:
                pending["amcl_seen"] = True

    def _battery_callback(self, message: BatteryState):
        with self._lock:
            self._telemetry["battery"] = {
                "voltage": self._finite_or_none(message.voltage),
                "current": self._finite_or_none(message.current),
                "percentage": self._finite_or_none(message.percentage),
                "temperature": self._finite_or_none(message.temperature),
            }

    def _body_state_callback(self, message: RobotStatus):
        with self._lock:
            self._telemetry["body_state"] = {
                "ctrl_mode": int(message.ctrl_mode_msg),
                "robot_mode": int(message.robot_mode_msg),
                "error": int(message.error_msg),
                "warning": int(message.warning_msg),
            }

    def _imu_callback(self, message: Imu):
        quaternion = message.orientation
        with self._lock:
            self._telemetry["imu"] = {
                "roll": self._roll_from_quaternion(quaternion),
                "pitch": self._pitch_from_quaternion(quaternion),
                "yaw": _yaw_from_quaternion(quaternion),
                "angular_velocity": {
                    "x": float(message.angular_velocity.x),
                    "y": float(message.angular_velocity.y),
                    "z": float(message.angular_velocity.z),
                },
            }

    def _motors_callback(self, message: LegMotors):
        self._hardware.mark_message("diablo")
        with self._lock:
            self._telemetry["motors"] = {
                "left_wheel": {
                    "position": float(message.left_wheel_pos),
                    "velocity": float(message.left_wheel_vel),
                    "revolutions": int(message.left_wheel_enc_rev),
                },
                "right_wheel": {
                    "position": float(message.right_wheel_pos),
                    "velocity": float(message.right_wheel_vel),
                    "revolutions": int(message.right_wheel_enc_rev),
                },
                "left_leg_length": float(message.left_leg_length),
                "right_leg_length": float(message.right_leg_length),
            }

    def _joint_state_callback(self, message: JointState):
        arm_joint_names = {
            definition["name"] for definition in JOINT_DEFINITIONS.values()
        }
        if any(str(name) in arm_joint_names for name in message.name):
            self._hardware.mark_message("dynamixel")
        with self._lock:
            for name, position in zip(message.name, message.position):
                if math.isfinite(float(position)):
                    self._joint_positions[str(name)] = float(position)

    def _update_tf_pose(self):
        # A tf2 buffer can retain the last map->base transform for a short
        # time after AMCL/Nav2 exits.  Once localization is no longer owned
        # by this web node, that transform is stale; prefer the live filtered
        # odometry immediately so the HMI does not appear frozen or lagging.
        with self._lock:
            odom_pose = copy.deepcopy(self._odom_pose)
            amcl_recent = (
                self._last_amcl_pose_time > 0.0
                and time.monotonic() - self._last_amcl_pose_time <= 3.0
            )
        localization_running = (
            self._hardware.process_status("localization")["active"]
            or self._hardware.process_status("navigation")["active"]
        )
        if odom_pose is not None and not localization_running and not amcl_recent:
            with self._lock:
                self._pose = odom_pose
            return

        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except Exception:
            transform = self._cached_transform(self.map_frame, self.base_frame)
            if transform is not None:
                pose = {
                    "x": float(transform["x"]),
                    "y": float(transform["y"]),
                    "theta": float(transform["yaw"]),
                    "source": "amcl_tf",
                }
                with self._lock:
                    self._pose = pose
                    self._last_tf_time = time.monotonic()
                return
            with self._lock:
                # Navigation owns the embedded map_server + AMCL launch.  Do
                # not fall back to odometry while either standalone
                # localization or Navigation is active: odom is in a
                # different frame and would make the robot icon jump on the
                # map whenever map->odom has not been published yet.
                localization_running = (
                    self._hardware.process_status("localization")["active"]
                    or self._hardware.process_status("navigation")["active"]
                )
                amcl_recent = time.monotonic() - self._last_amcl_pose_time <= 3.0
                if (
                    self._odom_pose is not None
                    and not localization_running
                    and not amcl_recent
                    and (
                        self._pose is None
                        or self._pose.get("source")
                        in ("odom", "wheel_odom", "filtered_odom")
                    )
                ):
                    self._pose = self._odom_pose
            return

        if isinstance(transform, dict):
            pose = {
                "x": float(transform["x"]),
                "y": float(transform["y"]),
                "theta": float(transform["yaw"]),
                "source": "amcl_tf",
            }
        else:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pose = {
                "x": float(translation.x),
                "y": float(translation.y),
                "theta": _yaw_from_quaternion(rotation),
                "source": self.map_frame,
            }
        with self._lock:
            self._pose = pose
            self._last_tf_time = time.monotonic()

    # -------------------- Web state and command API --------------------

    def snapshot(self, previous_versions=None):
        """Build a JSON-ready state packet without exposing mutable state."""
        with self._lock:
            versions = dict(self._versions)
            processes = self._hardware.process_snapshots()
            configured_commands = {
                "localization": self.localization_start_command,
                "navigation": self.navigation_start_command,
                "mapping": self.mapping_start_command,
            }
            for name, command in configured_commands.items():
                if not command and not processes[name]["active"]:
                    processes[name].update(
                        {
                            "state": "not_configured",
                            "message": "Launch command not configured",
                        }
                    )

            # The full Navigation launch owns map_server and AMCL.  It first
            # stops the standalone Localization launch to avoid duplicate
            # lifecycle nodes and competing map->odom broadcasters.  Expose
            # that embedded AMCL as an active localization component in the
            # HMI, otherwise the operator sees "LOCALIZATION OFF" while AMCL
            # is actually running inside Nav2.
            if processes["navigation"]["active"] and not processes["localization"]["active"]:
                processes["localization"] = {
                    "name": "localization",
                    "state": "embedded",
                    "active": True,
                    "pid": processes["navigation"].get("pid"),
                    "message": "AMCL + map_server berjalan di dalam Navigation",
                    "owner": "navigation",
                }
            state = {
                "type": "state",
                "stamp": time.time(),
                "pose": copy.deepcopy(self._pose),
                "wheel_pose": copy.deepcopy(self._odom_pose),
                "wheel_trajectory": copy.deepcopy(self._wheel_trajectory),
                "telemetry": copy.deepcopy(self._telemetry),
                "control_mode": self._control_mode,
                "nav_goal": self.get_nav_goal_status(),
                "hardware": self._hardware.snapshot(),
                "processes": processes,
                "joints": self.joint_status(),
                "mapping": self.mapping_status(),
                "map_status": self.map_status(),
                "versions": versions,
                "footprint": copy.deepcopy(self._footprint),
                "navigation_readiness": self.navigation_readiness(),
                "command_pipeline": self.command_pipeline_status(),
            }
            for key, value in (
                ("map", self._map),
                ("local_costmap", self._local_costmap),
                ("global_costmap", self._global_costmap),
                ("path", self._path),
                ("scan", self._scan),
            ):
                if previous_versions is None or previous_versions.get(key) != versions[key]:
                    state[key] = copy.deepcopy(value)
        return state

    def publish_manual_command(
        self,
        forward=0.0,
        left=0.0,
        roll=0.0,
        up=None,
        pitch=0.0,
        mode_mark=False,
        height_ctrl_mode=False,
        pitch_ctrl_mode=False,
        roll_ctrl_mode=False,
        stand_mode=False,
        jump_mode=False,
        split_mode=False,
        activate_manual=True,
        require_hardware=True,
    ):
        if require_hardware and not self.hardware_ready():
            raise RuntimeError(
                "Manual motion is locked. Press START HARDWARE and wait for Diablo motor feedback."
            )
        if activate_manual and self._control_mode != "manual":
            self.set_control_mode("manual")

        command = MotionCtrl()
        command.mode_mark = bool(mode_mark)
        command.value.forward = self._clamp(forward, self.max_forward)
        command.value.left = self._clamp(left, self.max_turn)
        command.value.roll = self._clamp(roll, self.max_roll)
        command.value.up = self.default_up if up is None else float(up)
        command.value.pitch = float(pitch)
        command.value.leg_split = 0.0
        command.mode.height_ctrl_mode = bool(height_ctrl_mode)
        command.mode.pitch_ctrl_mode = bool(pitch_ctrl_mode)
        command.mode.roll_ctrl_mode = bool(roll_ctrl_mode)
        command.mode.stand_mode = bool(stand_mode)
        command.mode.jump_mode = bool(jump_mode)
        command.mode.split_mode = bool(split_mode)
        self._manual_publisher.publish(command)

    def publish_stand_command(self, stand):
        self.set_control_mode("manual")
        self.publish_manual_command(
            up=self.default_up,
            mode_mark=True,
            stand_mode=bool(stand),
            activate_manual=False,
        )

    def publish_stop(self):
        self.set_control_mode("manual")
        self.publish_manual_command(activate_manual=False, require_hardware=False)

    def set_control_mode(self, mode):
        clean_mode = str(mode).strip().lower()
        if clean_mode not in ("manual", "auto", "stop"):
            raise ValueError("control mode must be manual, auto or stop")
        with self._lock:
            self._control_mode = clean_mode
        self._control_mode_publisher.publish(String(data=clean_mode))
        return clean_mode

    def send_nav_goal(self, x, y, theta, force=False):
        x = float(x)
        y = float(y)
        theta = float(theta)
        if not all(math.isfinite(value) for value in (x, y, theta)):
            raise ValueError("goal coordinates must be finite")

        readiness = self.navigation_readiness()
        if not readiness["ready"] and not force:
            with self._nav_goal_lock:
                self._goal_sequence += 1
                sequence = self._goal_sequence
                message = readiness["message"]
                self._current_goal_handle = None
                self._nav_goal_status = {
                    "state": "blocked",
                    "message": message,
                    "seq": sequence,
                    "distance_remaining": None,
                }
            return {
                "accepted": False,
                "blocked": True,
                "seq": sequence,
                "message": message,
                "readiness": readiness,
            }
        if force and not readiness["checks"]["action_server"]["ready"]:
            return {
                "accepted": False,
                "forced": True,
                "message": "Nav2 action server belum tersedia; start Navigation dulu.",
                "readiness": readiness,
            }

        with self._nav_goal_lock:
            old_handle = self._current_goal_handle
            if old_handle is not None:
                try:
                    old_handle.cancel_goal_async()
                except Exception:
                    pass
            self._goal_sequence += 1
            sequence = self._goal_sequence
            self._current_goal_handle = None
            self._nav_goal_status = {
                "state": "waiting",
                "message": "Waiting for Nav2 action server",
                "seq": sequence,
                "distance_remaining": None,
            }

        self.set_control_mode("auto")
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self._set_nav_goal_status(
                "error", "Nav2 action server is not available", sequence
            )
            self.set_control_mode("manual")
            return {
                "accepted": False,
                "seq": sequence,
                "message": "Nav2 action server is not available",
            }

        goal = NavigateToPose.Goal()
        goal.pose = self._make_goal_pose(x, y, theta)
        try:
            future = self._nav_to_pose_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._goal_feedback(
                    sequence, feedback
                ),
            )
            future.add_done_callback(
                lambda done: self._goal_response(sequence, done)
            )
        except Exception as error:
            self._set_nav_goal_status("error", str(error), sequence)
            self.set_control_mode("manual")
            return {"accepted": False, "seq": sequence, "message": str(error)}

        return {"accepted": True, "seq": sequence, "message": "Goal submitted"}

    def cancel_nav_goal(self):
        with self._nav_goal_lock:
            handle = self._current_goal_handle
            sequence = self._goal_sequence
        if handle is None:
            self.set_control_mode("manual")
            self._set_nav_goal_status("idle", "No active navigation goal", sequence)
            return {"canceled": False, "message": "No active navigation goal"}
        try:
            handle.cancel_goal_async()
        except Exception as error:
            return {"canceled": False, "message": str(error)}
        self.set_control_mode("manual")
        self._set_nav_goal_status("canceled", "Navigation goal canceled", sequence)
        return {"canceled": True, "message": "Navigation goal canceled"}

    def set_initial_pose(self, x, y, theta):
        x = float(x)
        y = float(y)
        theta = float(theta)
        if not all(math.isfinite(value) for value in (x, y, theta)):
            raise ValueError("initial pose must be finite")
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        quaternion = _quaternion_from_yaw(theta)
        message.pose.pose.orientation.x = quaternion["x"]
        message.pose.pose.orientation.y = quaternion["y"]
        message.pose.pose.orientation.z = quaternion["z"]
        message.pose.pose.orientation.w = quaternion["w"]
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.20
        self._initial_pose_publisher.publish(message)
        # Show the operator's requested AMCL seed immediately.  Once AMCL
        # processes the laser scan, /amcl_pose (or map->base TF) replaces this
        # pending value with its corrected map pose.
        with self._lock:
            self._pose = {"x": x, "y": y, "theta": theta, "source": "amcl_initial"}
            self._pending_initial_pose = {
                "message": message,
                "started_at": time.monotonic(),
                "attempts": 1,
                "amcl_seen": False,
            }
            self._initial_pose_diagnostic = {
                "state": "pending",
                "message": "Menunggu AMCL menerbitkan /amcl_pose dan TF map→odom",
                "attempts": 1,
                "age": 0.0,
            }
        return {"published": True, "x": x, "y": y, "theta": theta}

    def reset_odom(self):
        """Request the local wheel odometry to reset its pose origin."""
        if not self._reset_odom_client.wait_for_service(timeout_sec=0.5):
            return {"requested": False, "message": "Local odometry reset service is unavailable"}
        self._reset_odom_client.call_async(Trigger.Request())
        with self._lock:
            self._odom_pose = {
                "x": 0.0,
                "y": 0.0,
                "theta": 0.0,
                "source": "filtered_odom",
            }
            if self._pose is None or self._pose.get("source") in (
                "odom", "wheel_odom", "filtered_odom"
            ):
                self._pose = copy.deepcopy(self._odom_pose)
            self._wheel_trajectory = [{"x": 0.0, "y": 0.0}]
        return {"requested": True, "message": "Local wheel odometry reset requested"}

    def reset_position(self):
        """Reset only the local x/y origin and preserve the current heading."""
        if not self._reset_position_client.wait_for_service(timeout_sec=0.5):
            return {"requested": False, "message": "Position reset service is unavailable"}
        self._reset_position_client.call_async(Trigger.Request())
        with self._lock:
            theta = float(self._odom_pose.get("theta", 0.0)) if self._odom_pose else 0.0
            self._odom_pose = {
                "x": 0.0,
                "y": 0.0,
                "theta": theta,
                "source": "filtered_odom",
            }
            if self._pose is None or self._pose.get("source") in (
                "odom", "wheel_odom", "filtered_odom"
            ):
                self._pose = copy.deepcopy(self._odom_pose)
            self._wheel_trajectory = [{"x": 0.0, "y": 0.0}]
        return {"requested": True, "message": "Local odometry position reset requested"}

    def reset_orientation(self):
        """Reset only the local heading and preserve the current x/y pose."""
        if not self._reset_orientation_client.wait_for_service(timeout_sec=0.5):
            return {"requested": False, "message": "Orientation reset service is unavailable"}
        self._reset_orientation_client.call_async(Trigger.Request())
        with self._lock:
            x = float(self._odom_pose.get("x", 0.0)) if self._odom_pose else 0.0
            y = float(self._odom_pose.get("y", 0.0)) if self._odom_pose else 0.0
            self._odom_pose = {
                "x": x,
                "y": y,
                "theta": 0.0,
                "source": "filtered_odom",
            }
            if self._pose is None or self._pose.get("source") in (
                "odom", "wheel_odom", "filtered_odom"
            ):
                self._pose = copy.deepcopy(self._odom_pose)
        return {"requested": True, "message": "Local odometry orientation reset requested"}

    def reset_encoder(self):
        """Reset the wheel odometry encoder reference without changing pose."""
        if self._reset_encoder_client is None or not self._reset_encoder_client.wait_for_service(timeout_sec=0.5):
            return {
                "requested": False,
                "message": "Wheel encoder reset service is unavailable",
            }
        self._reset_encoder_client.call_async(Trigger.Request())
        return {
            "requested": True,
            "message": "Wheel encoder reference reset requested",
        }

    def joint_status(self):
        """Return the ten arm joints exposed by the full-body controllers."""
        with self._lock:
            status = []
            for motor_id, definition in JOINT_DEFINITIONS.items():
                current = self._joint_positions.get(definition["name"])
                status.append(
                    {
                        "id": motor_id,
                        "label": definition["label"],
                        "side": definition["side"],
                        "name": definition["name"],
                        "min": definition["min"],
                        "max": definition["max"],
                        "position": current,
                        "available": current is not None,
                    }
                )
            return status

    def set_joint_position(self, motor_id, position):
        """Send one validated joint target to the corresponding arm controller."""
        try:
            clean_id = int(motor_id)
            target = float(position)
        except (TypeError, ValueError) as error:
            raise ValueError(f"Invalid Dynamixel target: {error}")
        if clean_id not in JOINT_DEFINITIONS:
            raise ValueError("Only Dynamixel IDs 1 through 10 are web-controlled")
        if not math.isfinite(target):
            raise ValueError("Joint position must be finite")
        if not self.hardware_ready():
            raise RuntimeError(
                "Joint motion is locked. Start Hardware and wait for feedback first."
            )

        definition = JOINT_DEFINITIONS[clean_id]
        with self._lock:
            if definition["name"] not in self._joint_positions:
                raise RuntimeError(
                    f"Dynamixel ID {clean_id} has no /joint_states feedback yet"
                )
        target = max(definition["min"], min(definition["max"], target))
        message = JointTrajectory()
        message.joint_names = [definition["name"]]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500_000_000
        message.points = [point]
        self._joint_publishers[definition["side"]].publish(message)
        return {
            "requested": True,
            "id": clean_id,
            "joint": definition["name"],
            "position": target,
            "message": f"{definition['label']} target sent",
        }

    def start_lidar(self):
        """Start LiDAR using a configured command or Empty/Trigger service."""
        if self.lidar_start_command:
            result = self._hardware.start_component("lidar", self.lidar_start_command)
            result["hardware"] = self._hardware.snapshot()
            return result
        client = None
        request = None
        if self._lidar_start_empty_client is not None and self._lidar_start_empty_client.wait_for_service(timeout_sec=0.25):
            client = self._lidar_start_empty_client
            request = Empty.Request()
        elif self._lidar_start_client is not None and self._lidar_start_client.wait_for_service(timeout_sec=0.25):
            client = self._lidar_start_client
            request = Trigger.Request()
        if client is None:
            message = (
                "LiDAR start service is unavailable: "
                f"{self.lidar_start_service or 'not configured'} (Empty or Trigger)"
            )
            self._hardware.mark_service_start("lidar", False, message)
            return {
                "requested": False,
                "message": message,
                "hardware": self._hardware.snapshot(),
            }
        client.call_async(request)
        self._hardware.mark_service_start("lidar", True, "LiDAR start requested")
        return {
            "requested": True,
            "message": "LiDAR start requested",
            "hardware": self._hardware.snapshot(),
        }

    def stop_lidar(self):
        """Stop the LiDAR motor service and any LiDAR process owned by the HMI."""
        service_requested = False
        client = None
        request = None
        if self._lidar_stop_empty_client is not None and self._lidar_stop_empty_client.wait_for_service(timeout_sec=0.25):
            client = self._lidar_stop_empty_client
            request = Empty.Request()
        elif self._lidar_stop_client is not None and self._lidar_stop_client.wait_for_service(timeout_sec=0.25):
            client = self._lidar_stop_client
            request = Trigger.Request()
        if client is not None:
            # The motor-stop service is handled by the driver process itself.
            # Wait briefly for the DDS request to complete before terminating
            # the launch group; otherwise SIGTERM can race the callback and
            # leave the scanner motor spinning.
            future = client.call_async(request)
            deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            service_requested = True
        process_result = self._hardware.stop_process("lidar")
        return {
            "requested": bool(service_requested or process_result.get("requested")),
            "message": "LiDAR stop requested" if service_requested or process_result.get("requested") else "LiDAR was not running",
            "service_requested": service_requested,
            "process": process_result,
        }

    def start_hardware(self):
        """Start the configured Diablo, LiDAR and Dynamixel processes."""
        result = self._hardware.start_all()
        if not self.lidar_start_command:
            lidar_result = self.start_lidar()
            result.setdefault("results", {})["lidar_service"] = lidar_result
            result["requested"] = bool(result.get("requested") or lidar_result.get("requested"))
        result["hardware"] = self._hardware.snapshot()
        return result

    def _map_files(self, name):
        """Resolve a map YAML and its referenced image inside ``maps_dir``."""
        stem = _map_stem(name)
        if stem is None:
            raise ValueError("Invalid map name")
        root = self.maps_dir.expanduser().resolve()
        yaml_path = (root / f"{stem}.yaml").resolve()
        if root not in yaml_path.parents:
            raise ValueError("Invalid map path")
        if not yaml_path.is_file():
            raise FileNotFoundError(f"Map '{stem}.yaml' was not found")

        metadata = _read_map_yaml(yaml_path)
        image_value = metadata.get("image", f"{stem}.pgm")
        image_path = Path(str(image_value)).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()
        if root not in image_path.parents or not image_path.is_file():
            raise FileNotFoundError(
                f"Map '{yaml_path.name}' references missing image '{image_path.name}'"
            )
        return yaml_path, image_path, metadata

    def _selected_map_file(self):
        """Return the validated map selected by the HMI, if any."""
        marker = self.maps_dir / MAP_SELECTION_FILENAME
        try:
            raw = marker.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        if not raw:
            return None
        # The marker may contain an absolute path from an older web version;
        # only its basename is trusted so the source map directory remains
        # the single authority and paths cannot escape it.
        try:
            yaml_path, _image_path, _metadata = self._map_files(Path(raw).name)
        except (ValueError, FileNotFoundError, OSError):
            return None
        return yaml_path

    @staticmethod
    def _map_argument(command, argument, map_path):
        """Inject/replace a ROS launch map argument in a configured command."""
        command = str(command or "").strip()
        if not command or map_path is None:
            return command
        replacement = f"{argument}:={shlex.quote(str(map_path))}"
        # The built-in commands are simple ros2 launch invocations.  Replace
        # an existing argument so a stale map cannot win by appearing twice;
        # custom commands without the argument simply receive it at the end.
        pattern = rf"(?<!\S){re.escape(argument)}:=\S+"
        if re.search(pattern, command):
            return re.sub(pattern, replacement, command, count=1)
        return f"{command} {replacement}"

    def start_localization(self):
        map_path = self._selected_map_file()
        if map_path is None:
            return {
                "requested": False,
                "component": "localization",
                "message": (
                    "Localization dibatalkan: pilih map valid (.yaml + image) "
                    "sebelum menyalakan AMCL"
                ),
                "process": self._hardware.process_status("localization"),
            }
        # The standalone AMCL launch and full Nav2 launch own the same
        # map_server/amcl nodes.  Switch cleanly instead of creating duplicate
        # lifecycle nodes when the operator presses the other button.
        if self._hardware.process_status("navigation")["active"]:
            self.stop_navigation()
        command = self._map_argument(
            self.localization_start_command, "map_file", map_path
        )
        result = self._hardware.start_process("localization", command)
        return {**result, "component": "localization", "map_name": map_path.name}

    def start_navigation(self):
        map_path = self._selected_map_file()
        if map_path is None:
            return {
                "requested": False,
                "component": "navigation",
                "message": (
                    "Navigation dibatalkan: pilih map valid (.yaml + image) "
                    "sebelum menyalakan Nav2"
                ),
                "process": self._hardware.process_status("navigation"),
            }
        if self._hardware.process_status("localization")["active"]:
            self.stop_localization()
        with self._lock:
            self._navigation_lifecycle_active = False
            self._navigation_lifecycle_state = "waiting_for_amcl"
            self._navigation_lifecycle_last_request = 0.0
            # Discard visual data from a previous map before the new map
            # server publishes.  This prevents a stale corridor costmap/path
            # from being drawn over the newly selected map during startup.
            self._map = None
            self._local_costmap = None
            self._global_costmap = None
            self._path = None
            self._versions["map"] += 1
            self._versions["local_costmap"] += 1
            self._versions["global_costmap"] += 1
            self._versions["path"] += 1
            self._last_map_time = 0.0
            self._last_local_costmap_time = 0.0
            self._last_global_costmap_time = 0.0
        command = self._map_argument(self.navigation_start_command, "map", map_path)
        result = self._hardware.start_process("navigation", command)
        return {**result, "component": "navigation", "map_name": map_path.name}

    def start_mapping(self):
        if not self.hardware_mapping_ready():
            return {
                "requested": False,
                "component": "mapping",
                "message": (
                    "Mapping is locked until Diablo motors and LiDAR feedback are ready"
                ),
                "mapping": self.mapping_status(),
            }

        # SLAM Toolbox must be the only map->odom authority during mapping.
        # Stop the complete Nav2 stack (including its embedded AMCL) and any
        # standalone localization launch before starting SLAM.
        if self._hardware.process_status("navigation")["active"]:
            self.stop_navigation()
        if self._hardware.process_status("localization")["active"]:
            self.stop_localization()
        result = self._hardware.start_process("mapping", self.mapping_start_command)
        return {**result, "component": "mapping", "mapping": self.mapping_status()}

    def stop_mapping(self):
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(
                f"Could not stop robot before mapping shutdown: {error}"
            )
        result = self._hardware.stop_process("mapping")
        return {**result, "component": "mapping", "mapping": self.mapping_status()}

    def stop_localization(self):
        # When Navigation is active, AMCL belongs to that launch group rather
        # than to the standalone localization process.  Do not pretend that a
        # no-op stop succeeded; the operator must stop Navigation to disable
        # its embedded AMCL.
        if self._hardware.process_status("navigation")["active"]:
            return {
                "requested": False,
                "component": "localization",
                "embedded": True,
                "message": "AMCL is owned by Navigation; stop Navigation to disable localization",
                "process": self._hardware.process_status("localization"),
            }
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(
                f"Could not stop robot before localization shutdown: {error}"
            )
        result = self._hardware.stop_process("localization")
        with self._lock:
            self._last_amcl_pose_time = 0.0
            if self._odom_pose is not None:
                self._pose = copy.deepcopy(self._odom_pose)
        return {
            **result,
            "component": "localization",
            "process": self._hardware.process_status("localization"),
        }

    def stop_navigation(self):
        cancel = self.cancel_nav_goal()
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(
                f"Could not stop robot before navigation shutdown: {error}"
            )
        result = self._hardware.stop_process("navigation")
        with self._lock:
            self._last_amcl_pose_time = 0.0
            if self._odom_pose is not None:
                self._pose = copy.deepcopy(self._odom_pose)
            self._navigation_lifecycle_active = False
            self._navigation_lifecycle_state = "idle"
            self._navigation_lifecycle_start_future = None
            self._navigation_lifecycle_query_future = None
        return {
            **result,
            "component": "navigation",
            "cancel": cancel,
            "process": self._hardware.process_status("navigation"),
        }

    def stop_hardware(self):
        """Stop dependent web launches before stopping web-owned hardware."""
        stopped = {}
        for component, stopper in (
            ("mapping", self.stop_mapping),
            ("navigation", self.stop_navigation),
            ("localization", self.stop_localization),
        ):
            # Always issue the stop request.  Status can lag a timed-out
            # component (for example LiDAR) or a launch can still be in its
            # startup window; OFF HARDWARE must never be gated by that state.
            stopped[component] = stopper()
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(f"Could not publish hardware stop: {error}")
        lidar = self.stop_lidar()
        hardware = self._hardware.stop_hardware()
        return {
            "requested": bool(hardware.get("requested") or stopped or lidar.get("requested")),
            "message": "Hardware and dependent web launches stopped",
            "hardware": self._hardware.snapshot(),
            "lidar": lidar,
            "stopped": stopped,
            "results": hardware.get("results", {}),
            "external": hardware.get("external", {}),
        }

    def mapping_status(self):
        status = self._hardware.process_status("mapping")
        return {
            "state": status["state"],
            "active": bool(status["active"]),
            "message": status["message"],
            "pid": status["pid"],
        }

    def save_map(self, name):
        """Save the live SLAM Toolbox map as a PGM/YAML pair in diablo_bringup/map."""
        clean_name = str(name or "").strip()
        if not MAP_NAME_PATTERN.fullmatch(clean_name):
            raise ValueError(
                "Map name must start with a letter or number and contain only "
                "letters, numbers, '_' or '-' (max 64 characters)"
            )
        if not self.hardware_mapping_ready():
            return {
                "saved": False,
                "message": "Start Diablo and LiDAR before saving a map",
            }
        if self.mapping_status()["active"] is False:
            return {
                "saved": False,
                "message": "Start mapping before saving a map",
            }

        self.maps_dir.mkdir(parents=True, exist_ok=True)
        prefix = self.maps_dir / clean_name
        output_files = [prefix.with_suffix(".yaml"), prefix.with_suffix(".pgm")]
        existing = [path for path in output_files if path.exists()]
        if existing:
            return {
                "saved": False,
                "message": f"Map '{clean_name}' already exists; choose another name",
            }

        command = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(prefix),
            "--ros-args",
            "-p",
            "map_subscribe_transient_local:=true",
            "-p",
            "save_map_timeout:=10.0",
        ]
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=45.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return {"saved": False, "message": f"Map saver failed: {error}"}
        if completed.returncode != 0:
            details = (completed.stderr or completed.stdout or "").strip()
            return {
                "saved": False,
                "message": f"Map saver exited with code {completed.returncode}: {details[-500:]}",
            }
        missing = [str(path.name) for path in output_files if not path.is_file()]
        if missing:
            return {
                "saved": False,
                "message": f"Map saver finished but did not create: {', '.join(missing)}",
            }
        # A map saved by the mapping workflow is normally the map the operator
        # wants to localize with next.  Persist that choice immediately so a
        # later Navigation launch cannot silently fall back to an older map
        # such as Lab_corridor.  The operator can still choose another map in
        # the map picker before starting Nav2.
        try:
            selected = self.select_map(clean_name)
            selected_name = selected.get("map_name", output_files[0].name)
        except (ValueError, FileNotFoundError, RuntimeError) as error:
            return {
                "saved": True,
                "name": clean_name,
                "files": [path.name for path in output_files],
                "selected": False,
                "message": (
                    f"Map '{clean_name}' tersimpan, tetapi pemilihan map gagal: {error}"
                ),
            }
        return {
            "saved": True,
            "name": clean_name,
            "files": [path.name for path in output_files],
            "selected": True,
            "map_name": selected_name,
            "message": (
                f"Map '{clean_name}' saved to diablo_bringup/map and selected "
                "for the next localization/navigation launch"
            ),
        }

    def list_maps(self):
        """Return only complete PGM/YAML map pairs by name."""
        try:
            names = sorted(
                f"{item.stem}.pgm"
                for item in self.maps_dir.iterdir()
                if item.is_file()
                and item.suffix.lower() == ".yaml"
                and _map_stem(item.stem) is not None
            )
        except OSError:
            names = []
        complete = []
        for name in names:
            try:
                self._map_files(name)
            except (ValueError, FileNotFoundError, OSError):
                continue
            complete.append(name)
        return complete

    def select_map(self, name):
        """Persist the map that the next AMCL launch should load.

        The selected-map marker lives beside the map assets so a robot checkout
        remains self-contained.  Selecting a map does not restart AMCL; the UI
        can safely preview it first and the next localization launch consumes
        this marker.
        """
        requested = _map_stem(name)
        if requested is None:
            raise ValueError("Invalid map name")
        yaml_path, _image_path, _metadata = self._map_files(requested)

        root = self.maps_dir.expanduser().resolve()
        marker = root / MAP_SELECTION_FILENAME
        try:
            root.mkdir(parents=True, exist_ok=True)
            marker.write_text(f"{yaml_path.name}\n", encoding="utf-8")
        except OSError as error:
            raise RuntimeError(f"Could not persist selected map: {error}") from error
        navigation_active = self._hardware.process_status("navigation")["active"]
        localization_active = self._hardware.process_status("localization")["active"]
        return {
            "selected": True,
            "map_name": yaml_path.name,
            "restart_required": bool(navigation_active or localization_active),
            "message": (
                f"Map '{yaml_path.name}' disimpan untuk launch berikutnya. "
                + (
                    "Stop/restart Navigation atau Localization agar map aktif berganti."
                    if navigation_active or localization_active
                    else "Map ini akan dipakai saat Localization/Nav2 berikutnya dimulai."
                )
            ),
        }

    def selected_map(self):
        """Return the persisted map name, if it still exists."""
        candidate = self._selected_map_file()
        return candidate.name if candidate is not None else None

    def map_status(self):
        """Expose selected/live map metadata for diagnostics and the HMI."""
        selected = self.selected_map()
        with self._lock:
            live = self._map
        live_meta = None
        if live:
            origin = live.get("origin") or {}
            live_meta = {
                "frame_id": live.get("frame_id"),
                "width": int(live.get("width", 0)),
                "height": int(live.get("height", 0)),
                "resolution": float(live.get("resolution", 0.0)),
                "origin": {
                    "x": float(origin.get("x", 0.0)),
                    "y": float(origin.get("y", 0.0)),
                    "yaw": float(origin.get("yaw", 0.0)),
                },
            }
        return {
            "selected_map": selected,
            "maps_dir": str(self.maps_dir),
            "live": live_meta,
            "message": (
                f"Map aktif/terpilih: {selected}"
                if selected
                else "Belum ada map terpilih; pilih map sebelum Localization/Nav2"
            ),
        }

    def load_map(self, name):
        """Load a saved PGM/YAML map into the same JSON shape as /map."""
        requested = _map_stem(name)
        if requested is None:
            raise ValueError("Invalid map name")
        _yaml_path, pgm_path, metadata = self._map_files(requested)
        width, height, maximum, pixels = _read_pgm(pgm_path)
        resolution = float(metadata.get("resolution", 0.05))
        origin_values = metadata.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin_values, list) or len(origin_values) < 3:
            origin_values = [0.0, 0.0, 0.0]
        negate = bool(int(float(metadata.get("negate", 0))))
        occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
        free_threshold = float(metadata.get("free_thresh", 0.196))
        stride = max(1, math.ceil(math.sqrt((width * height) / MAX_MAP_CELLS)))
        parsed_width = math.ceil(width / stride)
        parsed_height = math.ceil(height / stride)
        # PGM pixels are stored from the top-left corner, while a ROS
        # OccupancyGrid stores row zero at the map origin (bottom-left).
        # Read the image bottom-to-top so preview clicks and the live
        # map_server /map use one coordinate convention.  The renderer then
        # applies the same single y-axis conversion to both grids.
        data = []
        for row in range(height - 1, -1, -stride):
            for column in range(0, width, stride):
                value = pixels[row * width + column]
                probability = value / maximum if negate else (maximum - value) / maximum
                if probability >= occupied_threshold:
                    data.append(100)
                elif probability <= free_threshold:
                    data.append(0)
                else:
                    data.append(-1)
        return {
            "name": f"{requested}.pgm",
            "frame_id": self.map_frame,
            "resolution": resolution * stride,
            "width": parsed_width,
            "height": parsed_height,
            "origin": {
                "x": float(origin_values[0]),
                "y": float(origin_values[1]),
                "yaw": float(origin_values[2]),
            },
            "data": data,
        }

    def hardware_status(self):
        return self._hardware.snapshot()

    def hardware_ready(self):
        return self._hardware.is_ready()

    def hardware_all_ready(self):
        return self._hardware.is_all_ready()

    def hardware_mapping_ready(self):
        return self._hardware.is_mapping_ready()

    def list_ros_topics(self):
        topics = []
        try:
            names_and_types = self.get_topic_names_and_types()
        except Exception:
            return topics
        for name, types in sorted(names_and_types, key=lambda item: item[0]):
            topics.append({"name": name, "types": list(types)})
        return topics

    def resolve_topic_types(self, topic_name):
        clean_name = self._normalize_topic_name(topic_name)
        return [item["types"] for item in self.list_ros_topics() if item["name"] == clean_name][0]

    def create_echo_subscription(self, topic_name, callback):
        clean_name = self._normalize_topic_name(topic_name)
        matches = [
            item for item in self.list_ros_topics() if item["name"] == clean_name
        ]
        if not matches or not matches[0]["types"]:
            raise ValueError(f"ROS topic not found: {clean_name}")
        type_name = matches[0]["types"][0]
        if "/action/" in type_name or "/srv/" in type_name:
            raise ValueError(f"Topic type is not a message: {type_name}")
        message_type = get_message(type_name)
        subscription = self.create_subscription(
            message_type,
            clean_name,
            callback,
            qos_profile_sensor_data,
        )
        return subscription, type_name

    def destroy_echo_subscription(self, subscription):
        if subscription is not None:
            self.destroy_subscription(subscription)

    def get_nav_goal_status(self):
        with self._nav_goal_lock:
            return copy.deepcopy(self._nav_goal_status)

    def nav2_ready(self):
        try:
            return bool(
                getattr(self, "_nav_to_pose_client", None)
                and self._nav_to_pose_client.server_is_ready()
            )
        except Exception:
            return False

    def navigation_readiness(self):
        """Return explicit prerequisites and diagnostics for a Nav2 goal.

        A goal is accepted by the normal web path only when the full robot
        data path is alive.  The separate FORCE button can still submit a
        goal when the action server exists, which is useful for diagnosis but
        is deliberately never selected automatically.
        """
        now = time.monotonic()
        navigation_process = self._hardware.process_status("navigation")
        selected_map_name = self.selected_map()
        try:
            hardware_ready = bool(self.hardware_ready())
        except Exception:
            hardware_ready = False
        with self._lock:
            map_value = self._map
            # These dictionaries are replaced atomically by the callbacks and
            # are never mutated after assignment.  A deep copy here used to
            # duplicate every costmap (often tens of thousands of cells) on
            # every WebSocket snapshot, even though readiness only needs a
            # few metadata fields.  Keep references and inspect them below.
            local_costmap = self._local_costmap
            global_costmap = self._global_costmap
            timestamps = {
                "map": self._last_map_time,
                "scan": self._last_scan_time,
                "odom": self._last_odom_time,
                "amcl": self._last_amcl_pose_time,
                "tf": self._last_tf_time,
                "local_costmap": self._last_local_costmap_time,
                "global_costmap": self._last_global_costmap_time,
            }
            pending_initial = self._pending_initial_pose is not None
            initial_diagnostic = copy.deepcopy(self._initial_pose_diagnostic)

        def age(last_time):
            if not last_time:
                return None
            return round(max(0.0, now - last_time), 2)

        def recent(last_time, limit):
            value = age(last_time)
            return value is not None and value <= limit

        map_to_odom = self._tf_available(self.map_frame, self.odom_frame)
        map_to_base = self._tf_available(self.map_frame, self.base_frame)
        action_server = self.nav2_ready()
        with self._lock:
            lifecycle_active = bool(self._navigation_lifecycle_active)
            lifecycle_state = str(self._navigation_lifecycle_state)
        # AMCL normally publishes /amcl_pose when it initializes and then
        # updates map->odom on every scan.  A stationary robot may therefore
        # have a perfectly valid TF while the last pose message is older than
        # the freshness window; TF is the authoritative readiness signal.
        amcl_ready = recent(timestamps["amcl"], 5.0) or map_to_odom
        local_ready = bool(
            local_costmap
            and (local_costmap.get("transform_ok") or local_costmap.get("frame_id") == self.map_frame)
            and recent(timestamps["local_costmap"], 4.0)
        )
        global_ready = bool(
            global_costmap
            and (global_costmap.get("transform_ok") or global_costmap.get("frame_id") == self.map_frame)
            and recent(timestamps["global_costmap"], 4.0)
        )
        checks = {
            "navigation_active": {
                "ready": bool(navigation_process.get("active")),
                "label": "Navigation launch",
                "age": None,
            },
            "navigation_lifecycle": {
                "ready": lifecycle_active,
                "label": "Nav2 lifecycle (BT/controller/smoother/collision monitor)",
                "state": lifecycle_state,
                "age": None,
            },
            "hardware": {
                "ready": hardware_ready,
                "label": "Hardware Diablo",
                "age": None,
            },
            "map": {
                "ready": map_value is not None,
                "label": "/map",
                "age": age(timestamps["map"]),
            },
            "map_selection": {
                "ready": selected_map_name is not None,
                "label": (
                    f"Selected map ({selected_map_name})"
                    if selected_map_name
                    else "Selected map"
                ),
                "age": None,
            },
            "scan": {
                "ready": recent(timestamps["scan"], 3.0),
                "label": self.scan_topic,
                "age": age(timestamps["scan"]),
            },
            "odom": {
                "ready": recent(timestamps["odom"], 3.0),
                "label": self.odom_topic,
                "age": age(timestamps["odom"]),
            },
            "amcl_pose": {
                "ready": amcl_ready,
                "label": "/amcl_pose",
                "age": age(timestamps["amcl"]),
                "via": "TF map→odom" if not recent(timestamps["amcl"], 5.0) and map_to_odom else "topic",
            },
            "tf_map_odom": {
                "ready": map_to_odom,
                "label": f"TF {self.map_frame}→{self.odom_frame}",
                "age": age(timestamps["tf"]),
            },
            "tf_map_base": {
                "ready": map_to_base,
                "label": f"TF {self.map_frame}→{self.base_frame}",
                "age": age(timestamps["tf"]),
            },
            "local_costmap": {
                "ready": local_ready,
                "label": "/local_costmap/costmap",
                "age": age(timestamps["local_costmap"]),
            },
            "global_costmap": {
                "ready": global_ready,
                "label": "/global_costmap/costmap",
                "age": age(timestamps["global_costmap"]),
            },
            "action_server": {
                "ready": action_server,
                "label": "/navigate_to_pose action",
                "age": None,
            },
        }
        blockers = [item["label"] for item in checks.values() if not item["ready"]]
        if blockers:
            message = "Navigation belum siap: " + ", ".join(blockers)
        else:
            message = "Navigation siap menerima goal normal"

        return {
            "ready": not blockers,
            "message": message,
            "blockers": blockers,
            "checks": checks,
            "force_allowed": action_server,
            "initial_pose": {
                **initial_diagnostic,
                "pending": pending_initial,
            },
            "pipeline": self.command_pipeline_status(),
            "local_costmap_window": {
                "width": float(local_costmap.get("width", 80) * local_costmap.get("resolution", 0.05))
                if local_costmap
                else 4.0,
                "height": float(local_costmap.get("height", 80) * local_costmap.get("resolution", 0.05))
                if local_costmap
                else 4.0,
                "frame_id": local_costmap.get("frame_id", self.odom_frame)
                if local_costmap
                else self.odom_frame,
            },
        }

    def command_pipeline_status(self):
        """Serialize observed Nav2 velocity pipeline values and ages."""
        now = time.monotonic()
        with self._lock:
            result = {}
            for key, item in self._command_pipeline.items():
                age = None if not item["last_time"] else round(max(0.0, now - item["last_time"]), 2)
                linear = float(item["linear"])
                angular = float(item["angular"])
                result[key] = {
                    "topic": item["topic"],
                    "age": age,
                    "recent": age is not None and age <= 3.0,
                    "nonzero": abs(linear) > 1e-4 or abs(angular) > 1e-4,
                    "linear": round(linear, 4),
                    "angular": round(angular, 4),
                }
            return result

    def _tf_available(self, target_frame, source_frame):
        if self._cached_transform(target_frame, source_frame) is not None:
            return True
        try:
            self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
            return True
        except Exception:
            return False

    def _cached_transform(self, target_frame, source_frame):
        """Return a recent cached 2-D transform, including simple chains."""
        target = str(target_frame or "").strip().lstrip("/")
        source = str(source_frame or "").strip().lstrip("/")
        if target == source:
            return {"x": 0.0, "y": 0.0, "yaw": 0.0, "received": time.monotonic()}
        now = time.monotonic()
        with self._lock:
            map_odom = copy.deepcopy(self._dynamic_tf.get("map_odom"))
            odom_base = copy.deepcopy(self._dynamic_tf.get("odom_base"))

        def recent(value):
            return value is not None and now - float(value.get("received", 0.0)) <= 3.0

        if target == self.map_frame and source == self.odom_frame:
            return map_odom if recent(map_odom) else None
        if target == self.odom_frame and source == self.base_frame:
            return odom_base if recent(odom_base) else None
        if target == self.map_frame and source == self.base_frame:
            if not (recent(map_odom) and recent(odom_base)):
                return None
            cosine = math.cos(map_odom["yaw"])
            sine = math.sin(map_odom["yaw"])
            return {
                "x": map_odom["x"] + cosine * odom_base["x"] - sine * odom_base["y"],
                "y": map_odom["y"] + sine * odom_base["x"] + cosine * odom_base["y"],
                "yaw": math.atan2(
                    math.sin(map_odom["yaw"] + odom_base["yaw"]),
                    math.cos(map_odom["yaw"] + odom_base["yaw"]),
                ),
                "received": min(map_odom["received"], odom_base["received"]),
            }
        if target == self.odom_frame and source == self.map_frame:
            if not recent(map_odom):
                return None
            cosine = math.cos(map_odom["yaw"])
            sine = math.sin(map_odom["yaw"])
            return {
                "x": -cosine * map_odom["x"] - sine * map_odom["y"],
                "y": sine * map_odom["x"] - cosine * map_odom["y"],
                "yaw": math.atan2(-math.sin(map_odom["yaw"]), math.cos(map_odom["yaw"])),
                "received": map_odom["received"],
            }
        return None

    def _update_hardware_status(self):
        try:
            topic_names = [name for name, _types in self.get_topic_names_and_types()]
            self._hardware.update(topic_names)
        except Exception as error:
            self.get_logger().debug(f"Hardware status refresh failed: {error}")

    def _retry_initial_pose(self):
        """Republish /initialpose during AMCL startup, then report timeout."""
        now = time.monotonic()
        with self._lock:
            pending = self._pending_initial_pose
            if pending is None:
                return
            elapsed = max(0.0, now - pending["started_at"])
            amcl_seen = bool(pending.get("amcl_seen"))
            if amcl_seen or self._tf_available(self.map_frame, self.odom_frame):
                self._pending_initial_pose = None
                self._initial_pose_diagnostic = {
                    "state": "confirmed",
                    "message": "AMCL / TF map→odom aktif",
                    "attempts": int(pending.get("attempts", 1)),
                    "age": round(elapsed, 2),
                }
                return
            if elapsed >= 8.0:
                attempts = int(pending.get("attempts", 1))
                self._pending_initial_pose = None
                self._initial_pose_diagnostic = {
                    "state": "timeout",
                    "message": (
                        "AMCL tidak menerbitkan /amcl_pose atau TF map→odom dalam 8 detik; "
                        "pastikan Navigation, /scan, odom, dan map aktif"
                    ),
                    "attempts": attempts,
                    "age": round(elapsed, 2),
                }
                self.get_logger().warning(self._initial_pose_diagnostic["message"])
                return
            message = pending["message"]
            try:
                self._initial_pose_publisher.publish(message)
            except Exception as error:
                self.get_logger().warning(f"Could not republish initial pose: {error}")
                return
            pending["attempts"] = int(pending.get("attempts", 1)) + 1
            self._initial_pose_diagnostic = {
                "state": "pending",
                "message": "Republish initial pose; menunggu AMCL / TF map→odom",
                "attempts": pending["attempts"],
                "age": round(elapsed, 2),
            }

    def _navigation_localization_ready(self):
        """Return true once AMCL's map->odom transform is observable."""
        if self._tf_available(self.map_frame, self.odom_frame):
            return True
        with self._lock:
            amcl_recent = (
                self._last_amcl_pose_time > 0.0
                and time.monotonic() - self._last_amcl_pose_time <= 5.0
            )
        return amcl_recent

    def _navigation_lifecycle_tick(self):
        """Start/query Nav2 lifecycle only after localization is ready.

        The navigation launch intentionally leaves its lifecycle manager in
        ``autostart:=false`` mode.  This callback is cheap and non-blocking:
        it sends one service request at a time and lets the ROS executor
        deliver the result asynchronously.
        """
        process = self._hardware.process_status("navigation")
        if not process.get("active"):
            with self._lock:
                self._navigation_lifecycle_active = False
                if self._navigation_lifecycle_state != "idle":
                    self._navigation_lifecycle_state = "idle"
            return

        now = time.monotonic()
        with self._lock:
            active = bool(self._navigation_lifecycle_active)
            state = self._navigation_lifecycle_state
            start_future = self._navigation_lifecycle_start_future
            query_future = self._navigation_lifecycle_query_future
            last_request = self._navigation_lifecycle_last_request
            last_query = self._navigation_lifecycle_last_query

        if start_future is not None and start_future.done():
            with self._lock:
                self._navigation_lifecycle_start_future = None
            try:
                response = start_future.result()
                success = bool(getattr(response, "success", False))
            except Exception as error:
                success = False
                self.get_logger().warning(
                    f"Nav2 lifecycle startup request failed: {error}"
                )
            with self._lock:
                self._navigation_lifecycle_active = success
                self._navigation_lifecycle_state = "active" if success else "error"
            if success:
                self.get_logger().info(
                    "Nav2 lifecycle activated after AMCL map→odom became available"
                )
            return

        if query_future is not None and query_future.done():
            with self._lock:
                self._navigation_lifecycle_query_future = None
            try:
                response = query_future.result()
                is_active = bool(getattr(response, "success", False))
            except Exception:
                is_active = False
            with self._lock:
                self._navigation_lifecycle_active = is_active
                if is_active:
                    self._navigation_lifecycle_state = "active"
            active = is_active

        # Keep the diagnostic state synchronized when an operator starts or
        # stops Navigation outside the web process.
        if not active and now - last_query >= 1.0:
            try:
                if self._navigation_lifecycle_active_client.service_is_ready():
                    future = self._navigation_lifecycle_active_client.call_async(
                        Trigger.Request()
                    )
                    with self._lock:
                        self._navigation_lifecycle_query_future = future
                        self._navigation_lifecycle_last_query = now
            except Exception:
                pass

        # Do not retry a manager that has already returned a failed STARTUP.
        # Nav2 lifecycle managers can partially activate earlier nodes before
        # reporting a later-node error; sending STARTUP again then attempts a
        # configure transition on active nodes and leaves the stack noisier and
        # harder to diagnose.  A fresh Navigation launch resets this state and
        # permits one clean retry.
        if (
            not active
            and state != "error"
            and start_future is None
            and now - last_request >= 2.0
            and self._navigation_localization_ready()
        ):
            self._request_navigation_lifecycle_startup(now)

    def _request_navigation_lifecycle_startup(self, now=None):
        """Request STARTUP on the Navigation lifecycle manager once."""
        now = time.monotonic() if now is None else now
        with self._lock:
            future = self._navigation_lifecycle_start_future
            last_request = self._navigation_lifecycle_last_request
        if future is not None and not future.done():
            return False
        if now - last_request < 2.0:
            return False
        try:
            if not self._navigation_lifecycle_client.service_is_ready():
                with self._lock:
                    self._navigation_lifecycle_state = "waiting_for_manager"
                return False
            request = ManageLifecycleNodes.Request()
            request.command = 0  # ManageLifecycleNodes::STARTUP
            future = self._navigation_lifecycle_client.call_async(request)
        except Exception as error:
            with self._lock:
                self._navigation_lifecycle_state = "error"
            self.get_logger().warning(f"Could not request Nav2 lifecycle startup: {error}")
            return False
        with self._lock:
            self._navigation_lifecycle_start_future = future
            self._navigation_lifecycle_last_request = now
            self._navigation_lifecycle_state = "starting"
        self.get_logger().info("Requested Nav2 lifecycle startup after AMCL readiness")
        return True

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        try:
            self._hardware.stop()
        except Exception as error:
            self.get_logger().warning(f"Could not stop HMI hardware processes: {error}")
        return super().destroy_node()

    # -------------------- Internal helpers --------------------

    def _goal_feedback(self, sequence, feedback_message):
        feedback = feedback_message.feedback
        distance = getattr(feedback, "distance_remaining", None)
        extra = {}
        if distance is not None:
            extra["distance_remaining"] = float(distance)
        self._set_nav_goal_status("navigating", "Nav2 is following the path", sequence, **extra)

    def _goal_response(self, sequence, future):
        try:
            handle = future.result()
        except Exception as error:
            self._set_nav_goal_status("error", str(error), sequence)
            self.set_control_mode("manual")
            return
        if not handle.accepted:
            self._set_nav_goal_status("rejected", "Nav2 rejected the goal", sequence)
            self.set_control_mode("manual")
            return

        with self._nav_goal_lock:
            if sequence != self._goal_sequence:
                return
            self._current_goal_handle = handle
            self._nav_goal_status["state"] = "accepted"
            self._nav_goal_status["message"] = "Nav2 accepted the goal"
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._goal_result(sequence, done)
        )

    def _goal_result(self, sequence, future):
        try:
            status = future.result().status
        except Exception as error:
            self._set_nav_goal_status("error", str(error), sequence)
            self.set_control_mode("manual")
            return

        labels = {
            GoalStatus.STATUS_SUCCEEDED: ("succeeded", "Goal reached"),
            GoalStatus.STATUS_CANCELED: ("canceled", "Goal canceled"),
            GoalStatus.STATUS_ABORTED: ("aborted", "Nav2 aborted the goal"),
        }
        state, message = labels.get(
            status, ("finished", f"Nav2 finished with status {status}")
        )
        with self._nav_goal_lock:
            if sequence != self._goal_sequence:
                return
            self._current_goal_handle = None
        self._set_nav_goal_status(state, message, sequence)
        self.set_control_mode("manual")

    def _set_nav_goal_status(self, state, message, sequence=None, **extra):
        with self._nav_goal_lock:
            if sequence is not None and sequence != self._goal_sequence:
                return
            self._nav_goal_status.update(
                {"state": state, "message": message, **extra}
            )

    def _make_goal_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        quaternion = _quaternion_from_yaw(theta)
        pose.pose.orientation.x = quaternion["x"]
        pose.pose.orientation.y = quaternion["y"]
        pose.pose.orientation.z = quaternion["z"]
        pose.pose.orientation.w = quaternion["w"]
        return pose

    def _parse_grid(self, message, max_cells=MAX_MAP_CELLS):
        """Serialize an OccupancyGrid in the map frame used by the HMI.

        Nav2 publishes the global costmap in ``map`` but the rolling local
        costmap in ``odom``.  Treating both origins as if they were already in
        map (the old Diablo renderer did this) makes the overlays appear
        shifted or completely outside the selected PGM.  Transform the grid
        origin just like the AMR web interface does; the cell values remain in
        their native grid and therefore retain their resolution/orientation.
        """
        origin = message.info.origin
        width = int(message.info.width)
        height = int(message.info.height)
        source = list(message.data)
        max_cells = max(1, int(max_cells))
        stride = max(1, math.ceil(math.sqrt((width * height) / max_cells)))
        if stride == 1:
            data = source
            parsed_width = width
            parsed_height = height
        else:
            parsed_width = math.ceil(width / stride)
            parsed_height = math.ceil(height / stride)
            data = []
            for row in range(0, height, stride):
                for column in range(0, width, stride):
                    values = []
                    for block_row in range(row, min(row + stride, height)):
                        start = block_row * width + column
                        values.extend(source[start : min(start + stride, block_row * width + width)])
                    if any(value >= 65 for value in values):
                        data.append(max(value for value in values if value >= 65))
                    elif any(value < 0 for value in values):
                        data.append(-1)
                    else:
                        data.append(max(values, default=0))
        source_frame = (message.header.frame_id or self.map_frame).strip().lstrip("/")
        transformed = self._transform_grid_origin_to_map(
            source_frame,
            float(origin.position.x),
            float(origin.position.y),
            _yaw_from_quaternion(origin.orientation),
        )
        return {
            "frame_id": transformed["frame_id"],
            "source_frame_id": source_frame,
            "target_frame_id": self.map_frame,
            "transform_ok": transformed["transform_ok"],
            "resolution": float(message.info.resolution) * stride,
            "width": parsed_width,
            "height": parsed_height,
            "origin": {
                "x": transformed["x"],
                "y": transformed["y"],
                "yaw": transformed["yaw"],
            },
            "data": data,
        }

    def _transform_grid_origin_to_map(self, source_frame, x, y, yaw):
        """Transform an occupancy-grid origin from its header frame to map."""
        if source_frame == self.map_frame:
            return {
                "frame_id": self.map_frame,
                "transform_ok": True,
                "x": x,
                "y": y,
                "yaw": yaw,
            }

        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, source_frame, Time()
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tf_yaw = _yaw_from_quaternion(rotation)
            cosine = math.cos(tf_yaw)
            sine = math.sin(tf_yaw)
            return {
                "frame_id": self.map_frame,
                "transform_ok": True,
                "x": float(translation.x) + cosine * x - sine * y,
                "y": float(translation.y) + sine * x + cosine * y,
                "yaw": math.atan2(math.sin(tf_yaw + yaw), math.cos(tf_yaw + yaw)),
            }
        except Exception as error:
            cached = self._cached_transform(self.map_frame, source_frame)
            if cached is not None:
                tf_yaw = float(cached["yaw"])
                cosine = math.cos(tf_yaw)
                sine = math.sin(tf_yaw)
                return {
                    "frame_id": self.map_frame,
                    "transform_ok": True,
                    "x": float(cached["x"]) + cosine * x - sine * y,
                    "y": float(cached["y"]) + sine * x + cosine * y,
                    "yaw": math.atan2(
                        math.sin(tf_yaw + yaw), math.cos(tf_yaw + yaw)
                    ),
                }
            now = time.monotonic()
            last_warn = self._last_grid_tf_warn.get(source_frame, 0.0)
            if now - last_warn > 5.0:
                with self._lock:
                    lifecycle_state = str(self._navigation_lifecycle_state)
                    initial_pending = self._pending_initial_pose is not None
                waiting_for_amcl = (
                    source_frame == self.odom_frame
                    and (
                        initial_pending
                        or lifecycle_state in {
                            "idle",
                            "waiting_for_amcl",
                            "waiting_for_manager",
                            "starting",
                        }
                    )
                )
                if waiting_for_amcl:
                    self.get_logger().info(
                        "Menunggu TF map→odom dari AMCL; kirim Set Initial Pose "
                        f"sebelum menggambar costmap (detail: {error})"
                    )
                else:
                    self.get_logger().warning(
                        f"TF {self.map_frame} <- {source_frame} unavailable for costmap: {error}"
                    )
                self._last_grid_tf_warn[source_frame] = now
            # Keep the source frame in the payload.  The frontend can avoid
            # drawing a misleading overlay until the transform becomes valid.
            return {
                "frame_id": source_frame,
                "transform_ok": False,
                "x": x,
                "y": y,
                "yaw": yaw,
            }

    @staticmethod
    def _default_maps_dir():
        """Prefer the source checkout's bringup map directory when available."""
        source_file = Path(__file__).resolve()
        for parent in (source_file.parent, *source_file.parents):
            candidate = parent / "src" / "diablo_bringup" / "map"
            if candidate.is_dir():
                return candidate
        try:
            from ament_index_python.packages import get_package_share_directory

            bringup_share = Path(get_package_share_directory("diablo_bringup"))
            installed_map = bringup_share / "map"
            if installed_map.is_dir():
                return installed_map
            legacy_map = bringup_share / "maps"
            if legacy_map.is_dir():
                return legacy_map
        except Exception:
            pass
        source_map = source_file.parent.parent / "maps"
        return source_map

    @staticmethod
    def _pose_from_pose_message(pose, source):
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "theta": _yaw_from_quaternion(pose.orientation),
            "source": source,
        }

    @staticmethod
    def _roll_from_quaternion(quaternion):
        sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1.0 - 2.0 * (
            quaternion.x * quaternion.x + quaternion.y * quaternion.y
        )
        return math.atan2(sinr_cosp, cosr_cosp)

    @staticmethod
    def _pitch_from_quaternion(quaternion):
        value = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        value = max(-1.0, min(1.0, value))
        return math.asin(value)

    @staticmethod
    def _finite_or_none(value):
        value = float(value)
        return value if math.isfinite(value) else None

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def _normalize_topic_name(topic_name):
        clean_name = str(topic_name or "").strip()
        if not clean_name:
            raise ValueError("ROS topic is required")
        if not clean_name.startswith("/"):
            clean_name = "/" + clean_name
        if ".." in clean_name or "//" in clean_name:
            raise ValueError("Invalid ROS topic name")
        return clean_name
