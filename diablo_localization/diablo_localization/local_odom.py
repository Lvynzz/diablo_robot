#!/usr/bin/env python3
"""Publish resettable local odometry from the ros2_control wheel odometry."""

import copy
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class LocalOdometry(Node):
    """Convert absolute wheel odometry into an operator-resettable local frame.

    The source odometry remains untouched.  A reset stores the current source
    pose as the new origin and publishes the relative planar pose from then
    on.  The local node owns ``odom -> base`` TF while it is active, so the raw
    diff-drive controller is configured not to publish that same transform.
    """

    def __init__(self):
        super().__init__("diablo_local_odom")

        self.declare_parameter("input_odom_topic", "/diablo_base_controller/odom")
        self.declare_parameter("output_odom_topic", "/diablo/odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "diablo_base_link")
        self.declare_parameter("reset_topic", "/diablo/reset_pose")
        self.declare_parameter("reset_service", "/diablo/reset_odom")
        self.declare_parameter(
            "stop_cmd_topic", "/diablo_base_controller/cmd_vel_unstamped"
        )
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("reset_on_start", False)

        input_topic = str(self.get_parameter("input_odom_topic").value).strip()
        output_topic = str(self.get_parameter("output_odom_topic").value).strip()
        self.odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        reset_topic = str(self.get_parameter("reset_topic").value).strip()
        reset_service = str(self.get_parameter("reset_service").value).strip()
        stop_cmd_topic = str(self.get_parameter("stop_cmd_topic").value).strip()
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.reset_on_start = bool(self.get_parameter("reset_on_start").value)

        if not input_topic or not output_topic or not self.odom_frame or not self.base_frame:
            raise ValueError("Odometry topics and frame names must not be empty")

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(Odometry, output_topic, reliable_qos)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._odom_callback, reliable_qos
        )
        self._reset_subscription = self.create_subscription(
            Bool, reset_topic, self._reset_topic_callback, 10
        )
        self._reset_service = self.create_service(
            Trigger, reset_service, self._reset_service_callback
        )
        self._stop_publisher = (
            self.create_publisher(Twist, stop_cmd_topic, 10)
            if stop_cmd_topic
            else None
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._latest_raw = None
        self._origin_set = False
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_yaw = 0.0
        self._received_reported = False

        self.get_logger().info(
            f"Local odometry ready: {input_topic} -> {output_topic}; "
            f"reset topic={reset_topic}, service={reset_service}, "
            f"publish_tf={self.publish_tf}, reset_on_start={self.reset_on_start}"
        )

    @staticmethod
    def _yaw_from_quaternion(orientation):
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quaternion_from_yaw(yaw):
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    def _odom_callback(self, message: Odometry):
        raw_x = float(message.pose.pose.position.x)
        raw_y = float(message.pose.pose.position.y)
        raw_yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
        if not all(math.isfinite(value) for value in (raw_x, raw_y, raw_yaw)):
            self.get_logger().error("Ignoring non-finite wheel odometry")
            return

        self._latest_raw = copy.deepcopy(message)
        if self.reset_on_start and not self._origin_set:
            self._set_origin(raw_x, raw_y, raw_yaw, "startup")

        local_x, local_y, local_yaw = self._local_pose(raw_x, raw_y, raw_yaw)
        local = copy.deepcopy(message)
        local.header.frame_id = self.odom_frame
        local.child_frame_id = self.base_frame
        local.pose.pose.position.x = local_x
        local.pose.pose.position.y = local_y
        local.pose.pose.position.z = 0.0
        local.pose.pose.orientation = self._quaternion_from_yaw(local_yaw)
        self._publisher.publish(local)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = local.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = local_x
            transform.transform.translation.y = local_y
            transform.transform.rotation = local.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

        if not self._received_reported:
            self.get_logger().info(
                f"Wheel odometry received at ({raw_x:.3f}, {raw_y:.3f}), "
                f"yaw={raw_yaw:.3f}; local pose=({local_x:.3f}, "
                f"{local_y:.3f}, {local_yaw:.3f})"
            )
            self._received_reported = True

    def _local_pose(self, raw_x, raw_y, raw_yaw):
        if not self._origin_set:
            return raw_x, raw_y, raw_yaw

        delta_x = raw_x - self._origin_x
        delta_y = raw_y - self._origin_y
        origin_cos = math.cos(self._origin_yaw)
        origin_sin = math.sin(self._origin_yaw)
        local_x = origin_cos * delta_x + origin_sin * delta_y
        local_y = -origin_sin * delta_x + origin_cos * delta_y
        local_yaw = self._normalize_angle(raw_yaw - self._origin_yaw)
        return local_x, local_y, local_yaw

    def _set_origin(self, raw_x, raw_y, raw_yaw, source):
        self._origin_x = raw_x
        self._origin_y = raw_y
        self._origin_yaw = raw_yaw
        self._origin_set = True
        self.get_logger().info(
            f"Local odometry reset by {source}: source pose "
            f"({raw_x:.3f}, {raw_y:.3f}, {raw_yaw:.3f}) is now (0, 0, 0)"
        )

    def _reset_topic_callback(self, message: Bool):
        if message.data:
            self._request_reset("topic command")

    def _reset_service_callback(self, _request, response):
        accepted, message = self._request_reset("service command")
        response.success = accepted
        response.message = message
        return response

    def _request_reset(self, source):
        if self._latest_raw is None:
            message = "No wheel odometry received yet"
            self.get_logger().error(message)
            return False, message

        raw = self._latest_raw
        raw_x = float(raw.pose.pose.position.x)
        raw_y = float(raw.pose.pose.position.y)
        raw_yaw = self._yaw_from_quaternion(raw.pose.pose.orientation)
        self._set_origin(raw_x, raw_y, raw_yaw, source)

        if self._stop_publisher is not None:
            self._stop_publisher.publish(Twist())

        # Publish an immediate exact-zero sample so a caller does not need to
        # wait for the next wheel feedback packet to observe the reset.
        local = copy.deepcopy(raw)
        local.header.frame_id = self.odom_frame
        local.child_frame_id = self.base_frame
        local.pose.pose.position.x = 0.0
        local.pose.pose.position.y = 0.0
        local.pose.pose.position.z = 0.0
        local.pose.pose.orientation = self._quaternion_from_yaw(0.0)
        self._publisher.publish(local)
        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = local.header
            transform.child_frame_id = self.base_frame
            transform.transform.rotation = local.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

        return True, "Local wheel odometry reset to (0, 0, 0)"


def main(args=None):
    rclpy.init(args=args)
    node = LocalOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
