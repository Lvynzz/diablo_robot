#!/usr/bin/env python3
"""Safety mux between web/manual and Nav2 Diablo motion commands."""

import copy
import threading
import time

import rclpy
from motion_msgs.msg import MotionCtrl
from rclpy.node import Node
from std_msgs.msg import String


class MotionCmdMux(Node):
    """Select one motion source and apply a command watchdog.

    The existing ``diablo_ctrl_node`` listens on ``/diablo/MotionCmd``.  This
    node keeps that public interface intact while making the two new inputs
    explicit:

    * ``/diablo/MotionCmd/manual`` for browser/keyboard teleoperation
    * ``/diablo/MotionCmd/nav`` for the Nav2 bridge

    The selected source must continue publishing.  If it goes quiet for
    ``command_timeout`` seconds, a zero command is sent to the controller.
    """

    def __init__(self):
        super().__init__("diablo_motion_cmd_mux")
        self.declare_parameter("manual_topic", "/diablo/MotionCmd/manual")
        self.declare_parameter("nav_topic", "/diablo/MotionCmd/nav")
        self.declare_parameter("output_topic", "/diablo/MotionCmd")
        self.declare_parameter("control_mode_topic", "/diablo/control_mode")
        self.declare_parameter("default_mode", "manual")
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("publish_frequency", 20.0)
        self.declare_parameter("default_up", 1.0)

        manual_topic = str(self.get_parameter("manual_topic").value)
        nav_topic = str(self.get_parameter("nav_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        mode_topic = str(self.get_parameter("control_mode_topic").value)
        default_mode = str(self.get_parameter("default_mode").value).strip().lower()
        self._mode = default_mode if default_mode in ("manual", "auto", "stop") else "manual"
        self._timeout = max(0.05, float(self.get_parameter("command_timeout").value))
        self._default_up = float(self.get_parameter("default_up").value)
        frequency = max(1.0, float(self.get_parameter("publish_frequency").value))

        self._lock = threading.Lock()
        self._manual_command = MotionCtrl()
        self._nav_command = MotionCtrl()
        self._manual_time = 0.0
        self._nav_time = 0.0

        self._publisher = self.create_publisher(MotionCtrl, output_topic, 10)
        self._manual_subscription = self.create_subscription(
            MotionCtrl, manual_topic, self._manual_callback, 10
        )
        self._nav_subscription = self.create_subscription(
            MotionCtrl, nav_topic, self._nav_callback, 10
        )
        self._mode_subscription = self.create_subscription(
            String, mode_topic, self._mode_callback, 10
        )
        self._timer = self.create_timer(1.0 / frequency, self._publish_selected)

        self.get_logger().info(
            f"Motion mux mode={self._mode}: manual={manual_topic}, nav={nav_topic}, "
            f"output={output_topic}"
        )

    def _manual_callback(self, msg):
        with self._lock:
            self._manual_command = copy.deepcopy(msg)
            self._manual_time = time.monotonic()

    def _nav_callback(self, msg):
        with self._lock:
            self._nav_command = copy.deepcopy(msg)
            self._nav_time = time.monotonic()

    def _mode_callback(self, msg: String):
        mode = str(msg.data).strip().lower()
        if mode not in ("manual", "auto", "stop"):
            self.get_logger().warning(f"Ignoring unknown control mode: {mode}")
            return
        with self._lock:
            self._mode = mode
        self.get_logger().info(f"Control mode: {mode}")

    def _publish_selected(self):
        now = time.monotonic()
        with self._lock:
            mode = self._mode
            manual = copy.deepcopy(self._manual_command)
            nav = copy.deepcopy(self._nav_command)
            manual_age = now - self._manual_time
            nav_age = now - self._nav_time

        if mode == "manual" and manual_age <= self._timeout:
            selected = manual
        elif mode == "auto" and nav_age <= self._timeout:
            selected = nav
        else:
            selected = self._zero_command()

        self._publisher.publish(selected)

    def _zero_command(self):
        command = MotionCtrl()
        command.mode_mark = False
        command.value.forward = 0.0
        command.value.left = 0.0
        command.value.up = self._default_up
        command.value.roll = 0.0
        command.value.pitch = 0.0
        command.value.leg_split = 0.0
        return command


def main(args=None):
    rclpy.init(args=args)
    node = MotionCmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
