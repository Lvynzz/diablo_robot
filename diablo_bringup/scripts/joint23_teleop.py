#!/usr/bin/env python3
"""
Teleop untuk 2 joint Dynamixel (ID2 dan ID3) lewat joint_trajectory_controller.

Cara pakai (setelah joint23_move.launch.py jalan):
    ros2 run diablo_bringup joint23_teleop.py

Format input: "<deg_joint2> <deg_joint3>"  (pisah spasi)
  - Angka derajat memakai skala yang SAMA seperti terlihat di Dynamixel Wizard (0-360).
  - Ketik 'x' pada salah satu posisi untuk membiarkan joint itu tetap di posisi sekarang.
    Contoh: "90 x"   -> hanya gerakkan joint2 ke 90 deg, joint3 diam.
  - Ketik 'c' untuk sekadar cek posisi saat ini tanpa menggerakkan apa pun.
  - Ketik 'q' untuk keluar.
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

JOINT_NAMES = ['joint2', 'joint3']
ACTION_NAME = '/joint23_trajectory_controller/follow_joint_trajectory'
DEFAULT_GOAL_DURATION_SEC = 0.7
# Beri satu siklus komunikasi untuk menyamakan state awal sebelum controller
# mulai mengejar target. Tanpa hold ini, perpindahan besar dapat melampaui
# trajectory tolerance pada ~0.5 s pertama ketika feedback masih terlambat.
STARTUP_HOLD_SEC = 0.5

# Offset antara pembacaan Wizard (raw tick, 0-360, raw=0 -> 0 deg) dan
# konvensi internal ros2_control (raw=2048/tengah -> 0 rad).
# Hasil kalibrasi dari ID2: wizard_deg = script_deg + WIZARD_OFFSET_DEG
# NOTE: asumsi offset yang sama berlaku untuk ID3 (konvensi ini berasal dari
# hardware_interface, bukan spesifik per motor). Kalau saat dites ID3 ternyata
# beda, kabari nilai kalibrasinya, nanti disesuaikan per-joint.
WIZARD_OFFSET_DEG = {
    'joint2': 180.0,
    'joint3': 180.0,
}

REST_POSITION_DEG = {
    'joint2': 264.3,
    'joint3': 177.8,  # isi setelah kamu cek resting position ID3 di Wizard
}


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wizard_deg_to_command_rad(joint_name: str, wizard_deg: float) -> float:
    script_deg = wizard_deg - WIZARD_OFFSET_DEG[joint_name]
    return deg2rad(script_deg)


def command_rad_to_wizard_deg(joint_name: str, rad: float) -> float:
    script_deg = rad2deg(rad)
    return (script_deg + WIZARD_OFFSET_DEG[joint_name]) % 360.0


class Joint23Teleop(Node):
    def __init__(self):
        super().__init__('joint23_teleop')
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._current_pos_rad = {name: None for name in JOINT_NAMES}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        for name in JOINT_NAMES:
            if name in msg.name:
                idx = msg.name.index(name)
                self._current_pos_rad[name] = msg.position[idx]

    def current_positions_deg(self):
        """dict {joint_name: wizard_deg atau None kalau belum ada data}"""
        result = {}
        for name in JOINT_NAMES:
            rad = self._current_pos_rad[name]
            result[name] = command_rad_to_wizard_deg(name, rad) if rad is not None else None
        return result

    def send_goal_deg(
        self, target_wizard_deg: dict, duration_sec: float = DEFAULT_GOAL_DURATION_SEC
    ):
        """
        target_wizard_deg: dict {joint_name: wizard_deg}. Joint yang tidak ada
        di dict ini tetap dimasukkan ke goal menggunakan feedback terakhir.
        Dengan begitu perintah "x" tidak bergantung pada perilaku partial-goal
        controller dan joint yang di-skip tetap dipertahankan posisinya.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server tidak ditemukan: {ACTION_NAME}')
            return

        if not target_wizard_deg:
            self.get_logger().warning('Tidak ada joint yang digerakkan (semua di-skip).')
            return

        # Selalu kirim kedua joint. Untuk slot "x", gunakan posisi feedback
        # terakhir sebagai target sehingga controller menerima trajectory yang
        # lengkap dan tidak perlu menebak posisi joint yang hilang.
        names = list(JOINT_NAMES)
        if any(self._current_pos_rad[name] is None for name in names):
            missing = [name for name in names if self._current_pos_rad[name] is None]
            self.get_logger().error(
                f'Feedback belum tersedia untuk {", ".join(missing)}; coba lagi setelah posisi tampil.'
            )
            return

        start_command_rad = {
            name: self._current_pos_rad[name] for name in names
        }
        target_command_rad = {}
        for name in names:
            if name in target_wizard_deg:
                target_command_rad[name] = wizard_deg_to_command_rad(
                    name, target_wizard_deg[name]
                )
            elif self._current_pos_rad[name] is not None:
                target_command_rad[name] = self._current_pos_rad[name]
            else:
                self.get_logger().error(
                    f'Feedback {name} belum tersedia; tidak dapat mempertahankan joint yang di-skip.'
                )
                return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = names

        start_point = JointTrajectoryPoint()
        start_point.positions = [start_command_rad[name] for name in names]
        start_point.time_from_start.sec = int(STARTUP_HOLD_SEC)
        start_point.time_from_start.nanosec = int(
            (STARTUP_HOLD_SEC - int(STARTUP_HOLD_SEC)) * 1e9
        )

        target_point = JointTrajectoryPoint()
        target_point.positions = [target_command_rad[name] for name in names]
        end_time_sec = STARTUP_HOLD_SEC + duration_sec
        target_point.time_from_start.sec = int(end_time_sec)
        target_point.time_from_start.nanosec = int(
            (end_time_sec - int(end_time_sec)) * 1e9
        )
        goal_msg.trajectory.points = [start_point, target_point]

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

        if not done_event.wait(timeout=STARTUP_HOLD_SEC + duration_sec + 5.0):
            self.get_logger().error('Timeout menunggu hasil goal dari controller.')


def format_current(positions: dict) -> str:
    parts = []
    for name in JOINT_NAMES:
        val = positions[name]
        parts.append(f'{name}={val:.2f}deg' if val is not None else f'{name}=menunggu...')
    return ', '.join(parts)


def main():
    rclpy.init()
    node = Joint23Teleop()

    def spin_node():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    print('Joint23 teleop - kontrol ID2 & ID3')
    print('Format input: "<deg_joint2> <deg_joint3>"  contoh: "90 120"')
    print('Ketik "x" di salah satu slot untuk skip joint itu, contoh: "90 x"')
    print('Ketik "c" untuk cek posisi saja, "q" untuk keluar.\n')

    try:
        while rclpy.ok():
            cur = node.current_positions_deg()
            user_in = input(f'[posisi sekarang: {format_current(cur)}] target (j2 j3) > ').strip()

            if user_in.lower() == 'q':
                break
            if user_in.lower() == 'c' or user_in == '':
                continue

            tokens = user_in.split()
            if len(tokens) != 2:
                print('Format salah. Contoh: "90 120" atau "90 x" (2 token dipisah spasi).')
                continue

            target = {}
            for name, tok in zip(JOINT_NAMES, tokens):
                if tok.lower() == 'x':
                    continue
                try:
                    value = float(tok)
                except ValueError:
                    print(f'Nilai "{tok}" untuk {name} tidak valid, harus angka atau "x".')
                    target = None
                    break
                if not 0.0 <= value <= 360.0:
                    print(f'Nilai {name} harus berada di antara 0 dan 360 derajat.')
                    target = None
                    break
                target[name] = value

            if not target:
                continue

            node.get_logger().info(
                f'Mengirim goal: {target} '
                f'(hold awal {STARTUP_HOLD_SEC:.1f}s, durasi gerak {DEFAULT_GOAL_DURATION_SEC:.1f}s)'
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
