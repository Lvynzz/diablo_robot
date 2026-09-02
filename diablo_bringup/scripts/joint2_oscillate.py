#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

REST_POSITION = 1.4726   # posisi rest terukur dari /joint_states
AMPLITUDE = 0.05          # +/- 0.05 rad (~2.9 derajat) -- sengaja kecil
PERIOD_SEC = 4.0          # satu ayunan penuh (kiri-kanan) selama 4 detik -- lambat
NUM_CYCLES = 3            # cuma 3 kali ayun, lalu berhenti
UPDATE_HZ = 20            # frekuensi publish command


class Joint2Oscillate(Node):
    def __init__(self):
        super().__init__('joint2_oscillate')
        self.pub = self.create_publisher(
            Float64MultiArray, '/joint2_position_controller/commands', 10)

    def send(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Joint2Oscillate()

    print(f'Rest position: {REST_POSITION:.4f} rad')
    print(f'Oscillating +/- {AMPLITUDE} rad for {NUM_CYCLES} cycles...')
    print('Tekan Ctrl+C kapan saja untuk berhenti dengan aman.')

    dt = 1.0 / UPDATE_HZ
    total_time = PERIOD_SEC * NUM_CYCLES
    steps = int(total_time / dt)

    try:
        for i in range(steps):
            t = i * dt
            offset = AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_SEC)
            target = REST_POSITION + offset
            node.send(target)
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\nDihentikan oleh user.')
    finally:
        # kembali ke posisi rest yang persis, lalu berhenti
        print('Kembali ke posisi rest...')
        node.send(REST_POSITION)
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
