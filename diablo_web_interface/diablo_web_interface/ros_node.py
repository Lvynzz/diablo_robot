#!/usr/bin/env python3
"""ROS 2 state, teleoperation and Nav2 integration for the Diablo web UI."""

from collections import OrderedDict
import copy
import math
from pathlib import Path
import re
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from motion_msgs.msg import LegMotors, MotionCtrl, RobotStatus
from nav2_msgs.action import NavigateToPose
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

from .hardware_manager import HardwareManager


MAX_ECHO_DEPTH = 5
MAX_ECHO_ITEMS = 80
MAX_LIDAR_POINTS = 720
MAX_MAP_CELLS = 250_000
MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
MAP_SELECTION_FILENAME = ".selected_localization_map.txt"


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
        self.declare_parameter("odom_topic", "/diablo/odometry")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "diablo_base_link")
        self.declare_parameter("map_frame", "map")
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
            "ros2 run diablo_ctrl diablo_ctrl_node "
            "--ros-args -p controller_port:=/dev/diablo_controller",
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
            "start_arm_controllers:=true start_base_controller:=false use_ekf:=false "
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
            "ros2 launch diablo_web_interface navigation.launch.py",
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

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._subscriptions = [
            self.create_subscription(
                OccupancyGrid, self.map_topic, self._map_callback, transient_qos
            ),
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self._local_costmap_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._global_costmap_callback,
                qos_profile_sensor_data,
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
                qos_profile_sensor_data,
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
        ]

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.1, self._update_tf_pose)
        self._hardware_timer = self.create_timer(0.5, self._update_hardware_status)
        self._nav_to_pose_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

        self.get_logger().info(
            f"Diablo web ROS node ready. Manual topic: {self.manual_cmd_topic}"
        )

    # -------------------- State callbacks --------------------

    def _map_callback(self, message):
        with self._lock:
            self._map = self._parse_grid(message)
            self._versions["map"] += 1

    def _local_costmap_callback(self, message):
        with self._lock:
            self._local_costmap = self._parse_grid(message)
            self._versions["local_costmap"] += 1

    def _global_costmap_callback(self, message):
        with self._lock:
            self._global_costmap = self._parse_grid(message)
            self._versions["global_costmap"] += 1

    def _odom_callback(self, message: Odometry):
        pose = self._pose_from_pose_message(message.pose.pose, "filtered_odom")
        with self._lock:
            self._odom_pose = pose
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
        points = [
            {
                "x": round(float(item.pose.position.x), 4),
                "y": round(float(item.pose.position.y), 4),
            }
            for item in message.poses
        ]
        with self._lock:
            self._path = {
                "frame_id": message.header.frame_id or self.map_frame,
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
        sensor_x = sensor_y = sensor_theta = 0.0
        try:
            scan_frame = message.header.frame_id or "laser"
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, scan_frame, Time()
            )
            sensor_x = float(transform.transform.translation.x)
            sensor_y = float(transform.transform.translation.y)
            sensor_theta = _yaw_from_quaternion(transform.transform.rotation)
        except Exception:
            # The static laser TF may not have arrived during the first scan.
            # Keep a base-frame fallback; subsequent scans retry the lookup.
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
            self._versions["scan"] += 1

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped):
        """Use AMCL's filtered map pose as the robot pose shown by the HMI."""
        pose = self._pose_from_pose_message(message.pose.pose, "amcl")
        with self._lock:
            self._pose = pose
            self._last_amcl_pose_time = time.monotonic()

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
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except Exception:
            with self._lock:
                localization_running = self._hardware.process_status("localization")["active"]
                amcl_recent = time.monotonic() - self._last_amcl_pose_time <= 3.0
                if self._odom_pose is not None and (
                    self._pose is None
                    or self._pose.get("source") in ("odom", "wheel_odom", "filtered_odom")
                    or (not localization_running and not amcl_recent)
                ):
                    self._pose = self._odom_pose
            return

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
                "versions": versions,
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

    def send_nav_goal(self, x, y, theta):
        x = float(x)
        y = float(y)
        theta = float(theta)
        if not all(math.isfinite(value) for value in (x, y, theta)):
            raise ValueError("goal coordinates must be finite")

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

    def start_localization(self):
        # The standalone AMCL launch and full Nav2 launch own the same
        # map_server/amcl nodes.  Switch cleanly instead of creating duplicate
        # lifecycle nodes when the operator presses the other button.
        if self._hardware.process_status("navigation")["active"]:
            self.stop_navigation()
        result = self._hardware.start_process(
            "localization", self.localization_start_command
        )
        return {**result, "component": "localization"}

    def start_navigation(self):
        if self._hardware.process_status("localization")["active"]:
            self.stop_localization()
        result = self._hardware.start_process("navigation", self.navigation_start_command)
        return {**result, "component": "navigation"}

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
        try:
            self.publish_stop()
        except Exception as error:
            self.get_logger().warning(
                f"Could not stop robot before localization shutdown: {error}"
            )
        result = self._hardware.stop_process("localization")
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
            if self._hardware.process_status(component)["active"]:
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
        return {
            "saved": True,
            "name": clean_name,
            "files": [path.name for path in output_files],
            "message": f"Map '{clean_name}' saved to diablo_bringup/map",
        }

    def list_maps(self):
        """Return PGM map assets by name without exposing filesystem paths."""
        try:
            names = sorted(
                item.name
                for item in self.maps_dir.iterdir()
                if item.is_file() and item.suffix.lower() == ".pgm"
            )
        except OSError:
            names = []
        return names

    def select_map(self, name):
        """Persist the map that the next AMCL launch should load.

        The selected-map marker lives beside the map assets so a robot checkout
        remains self-contained.  Selecting a map does not restart AMCL; the UI
        can safely preview it first and the next localization launch consumes
        this marker.
        """
        requested = str(name or "").strip()
        for suffix in (".pgm", ".yaml"):
            if requested.lower().endswith(suffix):
                requested = requested[: -len(suffix)]
                break
        if not MAP_NAME_PATTERN.fullmatch(requested):
            raise ValueError("Invalid map name")

        root = self.maps_dir.expanduser().resolve()
        pgm_path = (root / f"{requested}.pgm").resolve()
        yaml_path = (root / f"{requested}.yaml").resolve()
        if root not in pgm_path.parents or root not in yaml_path.parents:
            raise ValueError("Invalid map path")
        if not pgm_path.is_file() or not yaml_path.is_file():
            raise FileNotFoundError(f"Map '{requested}' requires both .pgm and .yaml")

        marker = root / MAP_SELECTION_FILENAME
        try:
            root.mkdir(parents=True, exist_ok=True)
            marker.write_text(f"{yaml_path.name}\n", encoding="utf-8")
        except OSError as error:
            raise RuntimeError(f"Could not persist selected map: {error}") from error
        return {
            "selected": True,
            "map_name": yaml_path.name,
            "message": (
                f"Map '{yaml_path.name}' disimpan untuk launch AMCL berikutnya. "
                "Restart localization bila AMCL sedang berjalan."
            ),
        }

    def selected_map(self):
        """Return the persisted map name, if it still exists."""
        marker = self.maps_dir / MAP_SELECTION_FILENAME
        try:
            raw = marker.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        if not raw:
            return None
        candidate = Path(raw)
        if not candidate.is_absolute():
            candidate = self.maps_dir / candidate.name
        if candidate.suffix.lower() == ".pgm":
            candidate = candidate.with_suffix(".yaml")
        try:
            candidate = candidate.resolve()
            root = self.maps_dir.expanduser().resolve()
            if root not in candidate.parents or not candidate.is_file():
                return None
        except OSError:
            return None
        return candidate.name

    def load_map(self, name):
        """Load a saved PGM/YAML map into the same JSON shape as /map."""
        requested = str(name or "").strip()
        if requested.lower().endswith(".pgm"):
            requested = requested[:-4]
        if not MAP_NAME_PATTERN.fullmatch(requested):
            raise ValueError("Invalid map name")
        root = self.maps_dir.expanduser().resolve()
        pgm_path = (root / f"{requested}.pgm").resolve()
        yaml_path = (root / f"{requested}.yaml").resolve()
        if root not in pgm_path.parents:
            raise ValueError("Invalid map path")
        if not pgm_path.is_file():
            raise FileNotFoundError(f"Map '{requested}.pgm' was not found")
        width, height, maximum, pixels = _read_pgm(pgm_path)
        metadata = {}
        if yaml_path.is_file():
            metadata = _read_map_yaml(yaml_path)
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
        data = []
        for row in range(0, height, stride):
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
            return bool(self._nav_to_pose_client.server_is_ready())
        except Exception:
            return False

    def _update_hardware_status(self):
        try:
            topic_names = [name for name, _types in self.get_topic_names_and_types()]
            self._hardware.update(topic_names)
        except Exception as error:
            self.get_logger().debug(f"Hardware status refresh failed: {error}")

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

    @staticmethod
    def _parse_grid(message):
        origin = message.info.origin
        width = int(message.info.width)
        height = int(message.info.height)
        source = list(message.data)
        stride = max(1, math.ceil(math.sqrt((width * height) / MAX_MAP_CELLS)))
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
        return {
            "frame_id": message.header.frame_id,
            "resolution": float(message.info.resolution) * stride,
            "width": parsed_width,
            "height": parsed_height,
            "origin": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "yaw": _yaw_from_quaternion(origin.orientation),
            },
            "data": data,
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
