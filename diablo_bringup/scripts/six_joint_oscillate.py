#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Urutan HARUS sama persis dengan urutan 'joints' di six_joint_controllers.yaml
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint6', 'joint7', 'joint8']

# Posisi rest terukur sebelumnya (dari log aktivasi hardware)
REST_POSITIONS = {
    'joint1': -3.13085,
    'joint2':  1.46035,
    'joint3':  0.0843689,
    'joint6': -0.0628932,
    'joint7':  0.0214757,
    'joint8': -0.00460194,
}

AMPLITUDE = 0.05      # +/- 0.05 rad, sama untuk semua joint
PERIOD_SEC = 4.0       # satu ayunan penuh selama 4 detik -- lambat
NUM_CYCLES = 3
UPDATE_HZ = 20


class SixJointOscillate(Node):
    def __init__(self):
        super().__init__('six_joint_oscillate')
        self.pub = self.create_publisher(
            Float64MultiArray, '/six_joint_position_controller/commands', 10)

    def send(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SixJointOscillate()

    print('Rest positions:')
    for name in JOINT_NAMES:
        print(f'  {name}: {REST_POSITIONS[name]:.4f} rad')
    print(f'Oscillating +/- {AMPLITUDE} rad, {NUM_CYCLES} cycles, semua joint bersamaan...')
    print('Tekan Ctrl+C kapan saja untuk berhenti dengan aman.')

    dt = 1.0 / UPDATE_HZ
    total_time = PERIOD_SEC * NUM_CYCLES
    steps = int(total_time / dt)

    try:
        for i in range(steps):
            t = i * dt
            offset = AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_SEC)
            positions = [REST_POSITIONS[name] + offset for name in JOINT_NAMES]
            node.send(positions)
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\nDihentikan oleh user.')
    finally:
        print('Kembali ke posisi rest...')
        node.send([REST_POSITIONS[name] for name in JOINT_NAMES])
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
