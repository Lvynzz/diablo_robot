#!/usr/bin/env python3
"""Convert Nav2 velocity commands to Diablo's ``MotionCtrl`` message."""

import copy
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from motion_msgs.msg import MotionCtrl
from rclpy.node import Node


class MotionCmdBridge(Node):
    """Translate a differential-drive ``Twist`` into Diablo motion fields.

    Diablo's SDK calls the forward speed ``value.forward`` and the yaw rate
    ``value.left``.  Despite the historical field name, ``left`` is the
    turning command used by the original Diablo keyboard teleop node.
    """

    def __init__(self):
        super().__init__("diablo_motion_cmd_bridge")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/diablo/MotionCmd/nav")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_forward_command", 1.0)
        self.declare_parameter("max_turn_command", 1.0)
        self.declare_parameter("default_up", 1.0)
        self.declare_parameter("publish_frequency", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.command_timeout = max(0.05, float(self.get_parameter("command_timeout").value))
        self.max_forward = abs(float(self.get_parameter("max_forward_command").value))
        self.max_turn = abs(float(self.get_parameter("max_turn_command").value))
        self.default_up = float(self.get_parameter("default_up").value)
        frequency = max(1.0, float(self.get_parameter("publish_frequency").value))

        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._last_command_time = 0.0

        self._publisher = self.create_publisher(MotionCtrl, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._twist_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / frequency, self._publish_command)

        self.get_logger().info(
            f"Nav2 Twist bridge: {input_topic} -> {output_topic}"
        )

    def _twist_callback(self, msg: Twist):
        with self._lock:
            self._last_twist = copy.deepcopy(msg)
            self._last_command_time = time.monotonic()

    def _publish_command(self):
        with self._lock:
            twist = copy.deepcopy(self._last_twist)
            age = time.monotonic() - self._last_command_time

        if age > self.command_timeout:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        command = MotionCtrl()
        command.mode_mark = False
        command.value.forward = self._clamp(twist.linear.x, self.max_forward)
        command.value.left = self._clamp(twist.angular.z, self.max_turn)
        command.value.up = self.default_up
        command.value.roll = 0.0
        command.value.pitch = 0.0
        command.value.leg_split = 0.0
        self._publisher.publish(command)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = MotionCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
