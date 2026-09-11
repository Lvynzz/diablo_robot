#!/usr/bin/env python3
"""Publish private raw wheel odometry for the Diablo EKF input."""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from motion_msgs.msg import LegMotors
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RawWheelOdometry(Node):
    """Integrate Diablo wheel telemetry without publishing an odom TF.

    The EKF is the only node allowed to publish ``odom -> diablo_base_link``.
    This source therefore emits an Odometry message only; its topic defaults
    to the private ``/diablo_base_controller/odom`` input used by the filter.
    """

    def __init__(self):
        super().__init__("diablo_raw_wheel_odom")
        self.declare_parameter("input_topic", "/diablo/sensor/Motors")
        self.declare_parameter("odom_topic", "/diablo_base_controller/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "diablo_base_link")
        self.declare_parameter("wheel_radius", 0.093)
        self.declare_parameter("track_width", 0.475)
        self.declare_parameter("left_wheel_direction", 1.0)
        self.declare_parameter("right_wheel_direction", 1.0)
        self.declare_parameter("use_encoder_revolutions", True)
        self.declare_parameter("max_wheel_delta", 1.5)

        input_topic = str(self.get_parameter("input_topic").value).strip()
        odom_topic = str(self.get_parameter("odom_topic").value).strip()
        self.odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        self.wheel_radius = abs(float(self.get_parameter("wheel_radius").value))
        self.track_width = abs(float(self.get_parameter("track_width").value))
        self.left_sign = float(self.get_parameter("left_wheel_direction").value)
        self.right_sign = float(self.get_parameter("right_wheel_direction").value)
        self.use_revolutions = bool(
            self.get_parameter("use_encoder_revolutions").value
        )
        self.max_wheel_delta = abs(
            float(self.get_parameter("max_wheel_delta").value)
        )
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
        self._subscription = self.create_subscription(
            LegMotors, input_topic, self._motor_callback, 10
        )
        self.get_logger().info(
            f"Raw wheel odom: {input_topic} -> {odom_topic}; "
            f"radius={self.wheel_radius:.3f} m, track={self.track_width:.3f} m, "
            "publish_tf=false"
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

        wheel_delta_left = left - self._last_left
        wheel_delta_right = right - self._last_right
        self._last_left = left
        self._last_right = right
        self._last_stamp_ns = stamp_ns
        if self.max_wheel_delta > 0.0 and (
            abs(wheel_delta_left) > self.max_wheel_delta
            or abs(wheel_delta_right) > self.max_wheel_delta
        ):
            self.get_logger().warning(
                "Ignored discontinuous wheel sample "
                f"(left={wheel_delta_left:.3f}, right={wheel_delta_right:.3f} rad)"
            )
            return

        delta_left = wheel_delta_left * self.wheel_radius
        delta_right = wheel_delta_right * self.wheel_radius
        distance = 0.5 * (delta_left + delta_right)
        delta_yaw = (delta_right - delta_left) / self.track_width
        self.x += distance * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += distance * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = self._normalize_angle(self.yaw + delta_yaw)

        left_velocity = (
            float(msg.left_wheel_vel) * self.left_sign * self.wheel_radius
        )
        right_velocity = (
            float(msg.right_wheel_vel) * self.right_sign * self.wheel_radius
        )
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self._quaternion_from_yaw(self.yaw)
        odom.twist.twist.linear.x = 0.5 * (left_velocity + right_velocity)
        odom.twist.twist.angular.z = (right_velocity - left_velocity) / self.track_width
        # The EKF fuses only x/y and forward velocity from this message.  Keep
        # all corresponding covariance entries finite and non-zero.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.10
        odom.twist.covariance[35] = 0.10
        self._publisher.publish(odom)

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
    node = RawWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
