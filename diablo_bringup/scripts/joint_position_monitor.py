#!/usr/bin/env python3
import curses
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointPositionMonitor(Node):
    def __init__(self, stdscr):
        super().__init__('joint_position_monitor')
        self.stdscr = stdscr
        self.latest = {}
        self.create_subscription(JointState, '/joint_states', self.callback, 10)
        curses.curs_set(0)
        self.stdscr.nodelay(True)

    def callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos
        self.draw()

    def draw(self):
        self.stdscr.erase()
        self.stdscr.addstr(0, 0, "Dynamixel Joint Positions  (Ctrl+C to quit)")
        self.stdscr.addstr(1, 0, "-" * 45)
        row = 2
        for name in sorted(self.latest.keys()):
            pos = self.latest[name]
            deg = math.degrees(pos)
            self.stdscr.addstr(row, 0, f"{name:10s}  {pos:8.4f} rad   {deg:8.2f} deg")
            row += 1
        self.stdscr.refresh()


def main(stdscr):
    rclpy.init()
    node = JointPositionMonitor(stdscr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
