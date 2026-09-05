#!/usr/bin/env python3
"""Reset a running robot_localization EKF to the local origin.

The reset is intentionally explicit.  The node does not reset at startup, so
an operator can move the robot to another test location without losing the
pose accumulated by the running estimator.
"""

from geometry_msgs.msg import Twist
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

        reset_topic = str(self.get_parameter("reset_topic").value).strip()
        reset_service = str(self.get_parameter("reset_service").value).strip()
        self.set_pose_service = str(
            self.get_parameter("set_pose_service").value
        ).strip()
        self.reset_frame = str(self.get_parameter("reset_frame").value).strip()
        stop_cmd_topic = str(self.get_parameter("stop_cmd_topic").value).strip()

        self._set_pose_client = self.create_client(SetPose, self.set_pose_service)
        self._stop_publisher = (
            self.create_publisher(Twist, stop_cmd_topic, 10)
            if stop_cmd_topic
            else None
        )
        self._reset_subscription = self.create_subscription(
            Bool, reset_topic, self._reset_topic_callback, 10
        )
        self._reset_service = self.create_service(
            Trigger, reset_service, self._reset_service_callback
        )

        self.get_logger().info(
            f"Pose reset ready: topic={reset_topic}, service={reset_service}, "
            f"EKF service={self.set_pose_service}; reset_on_start=False"
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
        if not self._set_pose_client.wait_for_service(timeout_sec=2.0):
            message = f"EKF set_pose service unavailable: {self.set_pose_service}"
            self.get_logger().error(message)
            return False, message

        if self._stop_publisher is not None:
            self._stop_publisher.publish(Twist())

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = self.reset_frame
        request.pose.pose.pose.position.x = 0.0
        request.pose.pose.pose.position.y = 0.0
        request.pose.pose.pose.position.z = 0.0
        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = 0.0
        request.pose.pose.pose.orientation.w = 1.0

        # Give the filter a confident planar reset while leaving unused axes
        # untouched by the two_d_mode configuration.
        request.pose.pose.covariance[0] = 1.0e-6
        request.pose.pose.covariance[7] = 1.0e-6
        request.pose.pose.covariance[35] = 1.0e-6

        future = self._set_pose_client.call_async(request)
        future.add_done_callback(self._set_pose_done)
        self.get_logger().info(
            f"Pose reset requested by {source}; new pose=(0, 0, 0) in {self.reset_frame}"
        )
        return True, "EKF pose reset requested"

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
