#!/usr/bin/env python3
"""Reset a running robot_localization EKF to the local origin.

The reset is intentionally explicit.  The node does not reset at startup, so
an operator can move the robot to another test location without losing the
pose accumulated by the running estimator.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_localization.srv import SetPose
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class PoseReset(Node):
    """Bridge a simple topic/service command to robot_localization/set_pose."""

    def __init__(self):
        super().__init__("diablo_pose_reset")

        self.declare_parameter("reset_topic", "/diablo/reset_pose")
        self.declare_parameter("reset_service", "/diablo/reset_odom")
        self.declare_parameter(
            # robot_localization creates this service in the node namespace.
            # In the ROS 2 Humble binary that means /set_pose; the node name
            # does not automatically become a namespace.
            "set_pose_service", "/set_pose"
        )
        self.declare_parameter("reset_frame", "odom")
        self.declare_parameter(
            "stop_cmd_topic", "/diablo_base_controller/cmd_vel_unstamped"
        )
        self.declare_parameter("filtered_odom_topic", "/odometry/filtered")
        self.declare_parameter(
            "reset_position_service", "/diablo/reset_position"
        )
        self.declare_parameter(
            "reset_orientation_service", "/diablo/reset_orientation"
        )

        reset_topic = str(self.get_parameter("reset_topic").value).strip()
        reset_service = str(self.get_parameter("reset_service").value).strip()
        self.set_pose_service = str(
            self.get_parameter("set_pose_service").value
        ).strip()
        self.reset_frame = str(self.get_parameter("reset_frame").value).strip()
        stop_cmd_topic = str(self.get_parameter("stop_cmd_topic").value).strip()
        filtered_odom_topic = str(
            self.get_parameter("filtered_odom_topic").value
        ).strip()
        reset_position_service = str(
            self.get_parameter("reset_position_service").value
        ).strip()
        reset_orientation_service = str(
            self.get_parameter("reset_orientation_service").value
        ).strip()

        self._set_pose_client = self.create_client(SetPose, self.set_pose_service)
        self._stop_publisher = (
            self.create_publisher(Twist, stop_cmd_topic, 10)
            if stop_cmd_topic
            else None
        )
        self._reset_subscription = self.create_subscription(
            Bool, reset_topic, self._reset_topic_callback, 10
        )
        self._odom_subscription = self.create_subscription(
            Odometry, filtered_odom_topic, self._odom_callback, 10
        )
        self._reset_service = self.create_service(
            Trigger, reset_service, self._reset_service_callback
        )
        self._reset_position_service = (
            self.create_service(
                Trigger,
                reset_position_service,
                self._reset_position_service_callback,
            )
            if reset_position_service
            else None
        )
        self._reset_orientation_service = (
            self.create_service(
                Trigger,
                reset_orientation_service,
                self._reset_orientation_service_callback,
            )
            if reset_orientation_service
            else None
        )

        self._latest_pose = None

        self.get_logger().info(
            f"Pose reset ready: topic={reset_topic}, service={reset_service}, "
            f"position={reset_position_service or 'disabled'}, "
            f"orientation={reset_orientation_service or 'disabled'}, "
            f"odom={filtered_odom_topic}, EKF service={self.set_pose_service}; "
            "reset_on_start=False"
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
    def _quaternion_from_yaw(yaw):
        return (
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def _odom_callback(self, message: Odometry):
        orientation = message.pose.pose.orientation
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        yaw = self._yaw_from_quaternion(orientation)
        if all(math.isfinite(value) for value in (x, y, yaw)):
            self._latest_pose = (x, y, yaw)

    def _reset_topic_callback(self, message: Bool):
        if message.data:
            self._request_reset("topic command", "all")

    def _reset_service_callback(self, _request, response):
        accepted, message = self._request_reset("service command", "all")
        response.success = accepted
        response.message = message
        return response

    def _reset_position_service_callback(self, _request, response):
        accepted, message = self._request_reset("position service", "position")
        response.success = accepted
        response.message = message
        return response

    def _reset_orientation_service_callback(self, _request, response):
        accepted, message = self._request_reset(
            "orientation service", "orientation"
        )
        response.success = accepted
        response.message = message
        return response

    def _request_reset(self, source, reset_kind):
        if not self._set_pose_client.wait_for_service(timeout_sec=2.0):
            message = f"EKF set_pose service unavailable: {self.set_pose_service}"
            self.get_logger().error(message)
            return False, message

        if self._stop_publisher is not None:
            self._stop_publisher.publish(Twist())

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = self.reset_frame
        current_x, current_y, current_yaw = self._latest_pose or (0.0, 0.0, 0.0)
        request.pose.pose.pose.position.x = (
            current_x if reset_kind == "orientation" else 0.0
        )
        request.pose.pose.pose.position.y = (
            current_y if reset_kind == "orientation" else 0.0
        )
        request.pose.pose.pose.position.z = 0.0
        reset_yaw = current_yaw if reset_kind == "position" else 0.0
        qz, qw = self._quaternion_from_yaw(reset_yaw)
        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = qz
        request.pose.pose.pose.orientation.w = qw

        # Give the filter a confident planar reset while leaving unused axes
        # untouched by the two_d_mode configuration.
        request.pose.pose.covariance[0] = 1.0e-6
        request.pose.pose.covariance[7] = 1.0e-6
        request.pose.pose.covariance[35] = 1.0e-6

        future = self._set_pose_client.call_async(request)
        future.add_done_callback(self._set_pose_done)
        self.get_logger().info(
            f"Pose reset requested by {source}; "
            f"new pose=({request.pose.pose.pose.position.x:.3f}, "
            f"{request.pose.pose.pose.position.y:.3f}, {reset_yaw:.3f}) "
            f"in {self.reset_frame}"
        )
        return True, f"EKF {reset_kind} pose reset requested"

    def _set_pose_done(self, future):
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS transport failure
            self.get_logger().error(f"EKF pose reset failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseReset()
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
