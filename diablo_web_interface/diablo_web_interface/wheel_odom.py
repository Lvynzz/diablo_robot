#!/usr/bin/env python3
"""Wheel odometry publisher for Diablo ``motion_msgs/LegMotors`` telemetry."""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from motion_msgs.msg import LegMotors
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class DiabloWheelOdom(Node):
    """Estimate planar odometry from the two Diablo wheel motors.

    The SDK exposes a revolution counter plus a position within one turn.
    This is useful as a starting odometry source for Nav2, but balancing-leg
    motion and wheel slip can make it less accurate than a fused estimator.
    Wheel signs and dimensions are parameters so they can be calibrated on
    the real robot without changing code.
    """

    def __init__(self):
        super().__init__("diablo_wheel_odom")
        self.declare_parameter("input_topic", "/diablo/sensor/Motors")
        self.declare_parameter("odom_topic", "/diablo/odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_radius", 0.105)
        self.declare_parameter("track_width", 0.3751)
        self.declare_parameter("left_wheel_direction", 1.0)
        self.declare_parameter("right_wheel_direction", 1.0)
        self.declare_parameter("use_encoder_revolutions", True)
        self.declare_parameter("publish_tf", True)

        input_topic = str(self.get_parameter("input_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.wheel_radius = abs(float(self.get_parameter("wheel_radius").value))
        self.track_width = abs(float(self.get_parameter("track_width").value))
        self.left_sign = float(self.get_parameter("left_wheel_direction").value)
        self.right_sign = float(self.get_parameter("right_wheel_direction").value)
        self.use_revolutions = bool(self.get_parameter("use_encoder_revolutions").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.wheel_radius <= 0.0 or self.track_width <= 0.0:
            raise ValueError("wheel_radius and track_width must be positive")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._initialized = False
        self._last_left = 0.0
        self._last_right = 0.0
        self._last_stamp_ns = 0

        self._publisher = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self._subscription = self.create_subscription(
            LegMotors, input_topic, self._motor_callback, 10
        )
        self._reset_service = self.create_service(
            Trigger, "/diablo/reset_odom", self._reset_callback
        )
        self._reset_encoder_service = self.create_service(
            Trigger, "/diablo/reset_encoder", self._reset_encoder_callback
        )

        self.get_logger().info(
            f"Wheel odom: {input_topic} -> {odom_topic}, "
            f"radius={self.wheel_radius:.3f} m, track={self.track_width:.3f} m"
        )

    def _motor_callback(self, msg: LegMotors):
        left = self._absolute_wheel_angle(msg.left_wheel_pos, msg.left_wheel_enc_rev)
        right = self._absolute_wheel_angle(msg.right_wheel_pos, msg.right_wheel_enc_rev)
        left *= self.left_sign
        right *= self.right_sign

        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            stamp = self.get_clock().now().to_msg()
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        if not self._initialized:
            self._last_left = left
            self._last_right = right
            self._last_stamp_ns = stamp_ns
            self._initialized = True
            return

        delta_left = (left - self._last_left) * self.wheel_radius
        delta_right = (right - self._last_right) * self.wheel_radius
        dt = (stamp_ns - self._last_stamp_ns) / 1_000_000_000.0
        if dt <= 0.0 or dt > 1.0:
            dt = 0.02

        self._last_left = left
        self._last_right = right
        self._last_stamp_ns = stamp_ns

        distance = 0.5 * (delta_left + delta_right)
        delta_yaw = (delta_right - delta_left) / self.track_width
        self.x += distance * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += distance * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = self._normalize_angle(self.yaw + delta_yaw)

        left_velocity = float(msg.left_wheel_vel) * self.left_sign * self.wheel_radius
        right_velocity = float(msg.right_wheel_vel) * self.right_sign * self.wheel_radius
        linear_velocity = 0.5 * (left_velocity + right_velocity)
        angular_velocity = (right_velocity - left_velocity) / self.track_width

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self._quaternion_from_yaw(self.yaw)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self._publisher.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

    def _reset_callback(self, _request, response):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._initialized = False
        response.success = True
        response.message = "Diablo wheel odometry reset"
        return response

    def _reset_encoder_callback(self, _request, response):
        self._initialized = False
        response.success = True
        response.message = "Diablo wheel encoder reference reset"
        return response

    def _absolute_wheel_angle(self, position, revolutions):
        if self.use_revolutions:
            return float(position) + float(revolutions) * 2.0 * math.pi
        return float(position)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quaternion_from_yaw(yaw):
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion


def main(args=None):
    rclpy.init(args=args)
    node = DiabloWheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
