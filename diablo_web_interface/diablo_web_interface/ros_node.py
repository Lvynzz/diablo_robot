#!/usr/bin/env python3
"""ROS 2 state, teleoperation and Nav2 integration for the Diablo web UI."""

from collections import OrderedDict
import copy
import math
from pathlib import Path
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
from tf2_ros import Buffer, TransformListener

from .hardware_manager import HardwareManager


MAX_ECHO_DEPTH = 5
MAX_ECHO_ITEMS = 80
MAX_LIDAR_POINTS = 720


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
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_forward_command", 1.0)
        self.declare_parameter("max_turn_command", 1.0)
        self.declare_parameter("max_roll_command", 0.2)
        self.declare_parameter("default_up", 1.0)
        self.declare_parameter("reset_encoder_service", "/diablo/reset_encoder")
        self.declare_parameter("lidar_start_service", "/start_motor")
        self.declare_parameter(
            "diablo_start_command", "ros2 run diablo_ctrl diablo_ctrl_node"
        )
        self.declare_parameter("lidar_start_command", "")
        self.declare_parameter(
            "dynamixel_start_command",
            "ros2 launch diablo_bringup six_joint_move.launch.py",
        )
        self.declare_parameter("hardware_log_directory", "/tmp")
        self.declare_parameter("localization_start_command", "")
        self.declare_parameter("navigation_start_command", "")
        self.declare_parameter("mapping_start_command", "")
        self.declare_parameter("maps_dir", "")

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
        self.localization_start_command = str(
            self.get_parameter("localization_start_command").value
        ).strip()
        self.navigation_start_command = str(
            self.get_parameter("navigation_start_command").value
        ).strip()
        self.mapping_start_command = str(
            self.get_parameter("mapping_start_command").value
        ).strip()
        configured_maps_dir = str(self.get_parameter("maps_dir").value).strip()
        if configured_maps_dir:
            self.maps_dir = Path(configured_maps_dir).expanduser()
        else:
            self.maps_dir = Path(__file__).resolve().parent.parent / "maps"
            if not self.maps_dir.is_dir():
                try:
                    from ament_index_python.packages import get_package_share_directory

                    self.maps_dir = Path(get_package_share_directory("diablo_web_interface")) / "maps"
                except Exception:
                    pass

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
        self._wheel_trajectory = []
        self._telemetry = {
            "battery": None,
            "body_state": None,
            "imu": None,
            "motors": None,
        }
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
        self._reset_odom_client = self.create_client(Trigger, "/diablo/reset_odom")
        self._reset_encoder_client = (
            self.create_client(Trigger, self.reset_encoder_service)
            if self.reset_encoder_service
            else None
        )
        self._lidar_start_client = (
            self.create_client(Trigger, self.lidar_start_service)
            if self.lidar_start_service
            else None
        )
        self._lidar_start_empty_client = (
            self.create_client(Empty, self.lidar_start_service)
            if self.lidar_start_service
            else None
        )
        self._hardware = HardwareManager(
            self.get_logger(),
            diablo_command=self.diablo_start_command,
            lidar_command=self.lidar_start_command,
            dynamixel_command=self.dynamixel_start_command,
            lidar_topic=self.scan_topic,
            log_directory=self.hardware_log_directory,
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
        pose = self._pose_from_pose_message(message.pose.pose, "wheel_odom")
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
            if self._pose is None or self._pose.get("source") in ("odom", "wheel_odom"):
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
        with self._lock:
            self._scan = {
                "frame_id": message.header.frame_id,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment) * stride,
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "ranges": clean_ranges,
            }
            self._versions["scan"] += 1

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

    def _joint_state_callback(self, _message: JointState):
        self._hardware.mark_message("dynamixel")

    def _update_tf_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except Exception:
            with self._lock:
                if self._odom_pose is not None:
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
        return {"published": True, "x": x, "y": y, "theta": theta}

    def reset_odom(self):
        """Request the optional wheel odometry reset service."""
        if not self._reset_odom_client.wait_for_service(timeout_sec=0.5):
            return {"requested": False, "message": "Wheel odometry reset service is unavailable"}
        self._reset_odom_client.call_async(Trigger.Request())
        with self._lock:
            self._odom_pose = {
                "x": 0.0,
                "y": 0.0,
                "theta": 0.0,
                "source": "wheel_odom",
            }
            self._wheel_trajectory = [{"x": 0.0, "y": 0.0}]
        return {"requested": True, "message": "Wheel odometry reset requested"}

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
        result = self._hardware.start_process(
            "localization", self.localization_start_command
        )
        return {**result, "component": "localization"}

    def start_navigation(self):
        result = self._hardware.start_process("navigation", self.navigation_start_command)
        return {**result, "component": "navigation"}

    def start_mapping(self):
        result = self._hardware.start_process("mapping", self.mapping_start_command)
        return {**result, "component": "mapping"}

    def list_maps(self):
        """Return map assets by name without exposing filesystem paths to the browser."""
        try:
            names = sorted(
                item.name
                for item in self.maps_dir.iterdir()
                if item.is_file() and item.suffix.lower() in (".yaml", ".yml", ".pgm")
            )
        except OSError:
            names = []
        return names

    def hardware_status(self):
        return self._hardware.snapshot()

    def hardware_ready(self):
        return self._hardware.is_ready()

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
            self.get_logger().debug("Hardware status refresh failed: %s", error)

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        try:
            self._hardware.stop()
        except Exception as error:
            self.get_logger().warning("Could not stop HMI hardware processes: %s", error)
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
        return {
            "frame_id": message.header.frame_id,
            "resolution": float(message.info.resolution),
            "width": int(message.info.width),
            "height": int(message.info.height),
            "origin": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "yaw": _yaw_from_quaternion(origin.orientation),
            },
            "data": list(message.data),
        }

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
