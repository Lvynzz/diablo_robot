#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class SimpleGoalController(Node):
    """Small go-to-(x,y) controller driven by diff_drive_controller odometry."""

    def __init__(self):
        super().__init__("simple_goal_controller")

        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("odom_timeout", 1.0)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("angular_gain", 1.5)
        self.declare_parameter("rotate_in_place_threshold", 0.60)
        # Humble's diff_drive_controller exposes relative topics below its
        # controller namespace.  Keep these defaults aligned with the
        # controller spawned by full_body_hardware.launch.py.
        self.declare_parameter(
            "cmd_vel_topic", "/diablo_base_controller/cmd_vel_unstamped"
        )
        self.declare_parameter("odom_topic", "/diablo_base_controller/odom")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.timer = self.create_timer(0.05, self.control_cycle)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = None
        self.goal_reported = False
        self.no_odom_reported = False

        self.get_logger().info(
            "Goal (%.3f, %.3f), tolerance %.3f m; waiting for odometry",
            self.goal_x,
            self.goal_y,
            self.goal_tolerance,
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, limit):
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, value))

    def parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == "goal_x":
                self.goal_x = float(parameter.value)
                self.goal_reported = False
            elif parameter.name == "goal_y":
                self.goal_y = float(parameter.value)
                self.goal_reported = False
        return SetParametersResult(successful=True)

    def odom_callback(self, message):
        self.x = message.pose.pose.position.x
        self.y = message.pose.pose.position.y

        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.last_odom_time = self.get_clock().now()
        self.no_odom_reported = False

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def control_cycle(self):
        if self.last_odom_time is None:
            self.publish_stop()
            if not self.no_odom_reported:
                self.get_logger().warn("No odometry received; publishing zero cmd_vel")
                self.no_odom_reported = True
            return

        odom_age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if self.odom_timeout > 0.0 and odom_age > self.odom_timeout:
            self.publish_stop()
            self.get_logger().error(
                "Odometry is stale (%.2f s); publishing zero cmd_vel", odom_age
            )
            return

        delta_x = self.goal_x - self.x
        delta_y = self.goal_y - self.y
        distance = math.hypot(delta_x, delta_y)
        if distance <= self.goal_tolerance:
            self.publish_stop()
            if not self.goal_reported:
                self.get_logger().info(
                    "Goal reached at (%.3f, %.3f)", self.x, self.y
                )
                self.goal_reported = True
            return

        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = self.normalize_angle(desired_heading - self.yaw)
        angular = self.clamp(
            self.angular_gain * heading_error, self.max_angular_speed
        )

        if abs(heading_error) > self.rotate_in_place_threshold:
            linear = 0.0
        else:
            linear = self.clamp(self.linear_gain * distance, self.max_linear_speed)

        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_vel_publisher.publish(command)

    def stop(self):
        self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
