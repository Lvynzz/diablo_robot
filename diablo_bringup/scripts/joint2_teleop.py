#!/usr/bin/env python3
"""
Teleop sederhana untuk 1 joint Dynamixel (ID 2) lewat joint_trajectory_controller.

Cara pakai (setelah joint2_move.launch.py jalan):
    ros2 run diablo_bringup joint2_teleop.py

Ketik angka derajat target lalu Enter untuk menggerakkan motor.
Ketik 'c' untuk sekadar cek posisi saat ini, 'q' untuk keluar.
"""

import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINT_NAME = 'joint2'
ACTION_NAME = '/joint2_trajectory_controller/follow_joint_trajectory'
REST_POSITION_DEG = 264.3  # posisi resting/referensi ID2


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


class Joint2Teleop(Node):
    def __init__(self):
        super().__init__('joint2_teleop')
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._current_pos_rad = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        if JOINT_NAME in msg.name:
            idx = msg.name.index(JOINT_NAME)
            self._current_pos_rad = msg.position[idx]

    def current_position_deg(self):
        if self._current_pos_rad is None:
            return None
        return rad2deg(self._current_pos_rad)

    def send_goal_deg(self, target_deg: float, duration_sec: float = 2.0):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server tidak ditemukan: {ACTION_NAME}')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [deg2rad(target_deg)]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal_msg.trajectory.points = [point]

        done_event = threading.Event()

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal ditolak oleh controller')
                done_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)

        def result_cb(future):
            result = future.result().result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info('Goal selesai: SUCCESSFUL')
            else:
                self.get_logger().error(
                    f'Goal gagal, error_code={result.error_code}, '
                    f'error_string="{result.error_string}"'
                )
            done_event.set()

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(goal_response_cb)

        # tunggu sampai goal selesai (dengan timeout jaga-jaga)
        done_event.wait(timeout=duration_sec + 5.0)


def main():
    rclpy.init()
    node = Joint2Teleop()

    # spin di background thread supaya /joint_states & action callback terus jalan
    def spin_node():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    print(f'Joint2 teleop - referensi resting position: {REST_POSITION_DEG:.1f} deg, positif ke arah dalam robot')
    print('Masukkan posisi target (derajat), "c" untuk cek posisi saat ini, "q" untuk keluar.\n')

    try:
        while rclpy.ok():
            cur = node.current_position_deg()
            cur_str = f'{cur:.2f} deg' if cur is not None else 'menunggu /joint_states...'
            user_in = input(f'[posisi sekarang: {cur_str}] target deg > ').strip()

            if user_in.lower() == 'q':
                break
            if user_in.lower() == 'c':
                continue
            try:
                target_deg = float(user_in)
            except ValueError:
                print('Input tidak valid. Masukkan angka, "c", atau "q".')
                continue

            node.get_logger().info(f'Mengirim goal: {target_deg:.2f} deg')
            node.send_goal_deg(target_deg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
