#!/usr/bin/env python3
"""Teleop trajectory untuk Dynamixel ID1, ID2, dan ID3.

Jalankan setelah joint123_move.launch.py aktif:
    ros2 run diablo_bringup joint123_teleop.py

Input memakai derajat yang sama seperti Dynamixel Wizard (0--360):
    <joint1> <joint2> <joint3>
    Contoh: 180 260 180

Gunakan ``x`` untuk mempertahankan joint pada posisi feedback terakhir:
    180 x 180

``c`` hanya menampilkan posisi, sedangkan ``q`` keluar.
"""

import math
import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = ['joint1', 'joint2', 'joint3']
ACTION_NAME = '/joint123_trajectory_controller/follow_joint_trajectory'
DEFAULT_GOAL_DURATION_SEC = 2.5
STARTUP_HOLD_SEC = 0.5

# xm430_w350.model memakai raw 2048 (Wizard 180 deg) sebagai 0 rad ROS.
WIZARD_OFFSET_DEG = {
    'joint1': 180.0,
    'joint2': 180.0,
    'joint3': 180.0,
}

REST_POSITION_DEG = {
    'joint1': 0.0,
    'joint2': 264.3,
    'joint3': 177.8,
}


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wizard_deg_to_command_rad(joint_name: str, wizard_deg: float) -> float:
    return deg2rad(wizard_deg - WIZARD_OFFSET_DEG[joint_name])


def command_rad_to_wizard_deg(joint_name: str, rad: float) -> float:
    return (rad2deg(rad) + WIZARD_OFFSET_DEG[joint_name]) % 360.0


class Joint123Teleop(Node):
    def __init__(self):
        super().__init__('joint123_teleop')
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._current_pos_rad = {name: None for name in JOINT_NAMES}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        for name in JOINT_NAMES:
            if name in msg.name:
                self._current_pos_rad[name] = msg.position[msg.name.index(name)]

    def current_positions_deg(self):
        return {
            name: (
                command_rad_to_wizard_deg(name, self._current_pos_rad[name])
                if self._current_pos_rad[name] is not None else None
            )
            for name in JOINT_NAMES
        }

    def send_goal_deg(
        self,
        target_wizard_deg: dict,
        duration_sec: float = DEFAULT_GOAL_DURATION_SEC,
    ):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server tidak ditemukan: {ACTION_NAME}')
            return

        missing = [
            name for name in JOINT_NAMES if self._current_pos_rad[name] is None
        ]
        if missing:
            self.get_logger().error(
                f'Feedback belum tersedia untuk {", ".join(missing)}; '
                'coba lagi setelah semua posisi tampil.'
            )
            return

        # Selalu kirim tiga joint. Joint dengan input x mempertahankan posisi
        # feedback terakhir sehingga controller menerima trajectory lengkap.
        start_positions = [self._current_pos_rad[name] for name in JOINT_NAMES]
        target_positions = [
            wizard_deg_to_command_rad(name, target_wizard_deg[name])
            if name in target_wizard_deg else self._current_pos_rad[name]
            for name in JOINT_NAMES
        ]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(JOINT_NAMES)

        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.time_from_start.sec = int(STARTUP_HOLD_SEC)
        start_point.time_from_start.nanosec = int(
            (STARTUP_HOLD_SEC - int(STARTUP_HOLD_SEC)) * 1e9
        )

        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        end_time_sec = STARTUP_HOLD_SEC + duration_sec
        target_point.time_from_start.sec = int(end_time_sec)
        target_point.time_from_start.nanosec = int(
            (end_time_sec - int(end_time_sec)) * 1e9
        )
        goal_msg.trajectory.points = [start_point, target_point]

        done_event = threading.Event()

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

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal ditolak oleh controller')
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(result_cb)

        self._client.send_goal_async(goal_msg).add_done_callback(goal_response_cb)
        if not done_event.wait(timeout=STARTUP_HOLD_SEC + duration_sec + 5.0):
            self.get_logger().error('Timeout menunggu hasil goal dari controller.')


def format_current(positions: dict) -> str:
    return ', '.join(
        f'{name}={positions[name]:.2f}deg'
        if positions[name] is not None else f'{name}=menunggu...'
        for name in JOINT_NAMES
    )


def main():
    rclpy.init()
    node = Joint123Teleop()

    def spin_node():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    print('Joint123 teleop - kontrol ID1, ID2 & ID3')
    print('Format input: "<deg_joint1> <deg_joint2> <deg_joint3>"')
    print('Contoh: "180 260 180"')
    print('Ketik "x" pada slot untuk skip joint, contoh: "180 x 180"')
    print('Ketik "c" untuk cek posisi saja, "q" untuk keluar.\n')

    try:
        while rclpy.ok():
            current = node.current_positions_deg()
            user_input = input(
                f'[posisi sekarang: {format_current(current)}] '
                'target (j1 j2 j3) > '
            ).strip()

            if user_input.lower() == 'q':
                break
            if user_input.lower() == 'c' or user_input == '':
                continue

            tokens = user_input.split()
            if len(tokens) != len(JOINT_NAMES):
                print('Format salah. Contoh: "180 260 180" atau "180 x 180".')
                continue

            target = {}
            valid = True
            for name, token in zip(JOINT_NAMES, tokens):
                if token.lower() == 'x':
                    continue
                try:
                    value = float(token)
                except ValueError:
                    print(f'Nilai "{token}" untuk {name} harus angka atau x.')
                    valid = False
                    break
                if not 0.0 <= value <= 360.0:
                    print(f'Nilai {name} harus berada di antara 0 dan 360 derajat.')
                    valid = False
                    break
                target[name] = value

            if not valid or not target:
                continue

            node.get_logger().info(
                f'Mengirim goal: {target} '
                f'(hold awal {STARTUP_HOLD_SEC:.1f}s, '
                f'durasi gerak {DEFAULT_GOAL_DURATION_SEC:.1f}s)'
            )
            node.send_goal_deg(target)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
