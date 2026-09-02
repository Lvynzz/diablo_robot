#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class JointTrajectoryMoveToPose(Node):
    def __init__(self):
        super().__init__('seven_joint_move_to_pose_node')
        
        # Publisher ke topik controller
        # JointTrajectoryController akan membaca topik ini dan menggerakkan dynamixel
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/seven_joint_trajectory_controller/joint_trajectory', 
            10)
            
        # Nama joint harus sama dengan yang ada di konfigurasi controller (urdf dan yaml)
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint6', 'joint7', 'joint8', 'joint11'
        ]

    def move_to_pose(self, target_positions, duration_sec):
        """
        Fungsi untuk menggerakkan joint ke target koordinat tertentu.
        target_positions: list dari 7 float (radian), merepresentasikan sudut tiap joint
        duration_sec: waktu yang diberikan untuk bergerak dari posisi saat ini ke target (detik)
        """
        
        # Validasi jumlah input posisi
        if len(target_positions) != len(self.joint_names):
            self.get_logger().error(f"Harus ada {len(self.joint_names)} posisi target!")
            return

        # 1. Buat pesan utama
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # 2. Buat titik tujuan (waypoint)
        point = JointTrajectoryPoint()
        # Masukkan koordinat tujuan
        point.positions = target_positions
        
        # Atur waktu yang dibutuhkan untuk mencapai titik tersebut.
        # Semakin lama durasinya, gerakan akan semakin pelan.
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # 3. Masukkan titik tersebut ke dalam pesan trajectory
        msg.points.append(point)
        
        # 4. Kirim (publish) pesan ke ROS2 Control
        self.get_logger().info(f"Mengirim gerakan ke koordinat: {target_positions} dalam {duration_sec} detik.")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryMoveToPose()

    # Tunggu sebentar agar koneksi publisher terbentuk dengan subscriber di controller_manager
    time.sleep(1)

    # Contoh 1: Bergerak ke resting position (berdasarkan info di file ID_dynamixel,
    # namun dikonversi ke radian atau format sesuai dynamixel hardware.
    # Di sini kita contohkan menggunakan angka 0.0 terlebih dahulu.)
    # Urutan: joint1, joint2, joint3, joint6, joint7, joint8, joint11
    target_pose_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Gerak ke pose 1 selama 2 detik
    node.move_to_pose(target_pose_1, 2.0)
    
    # Tunggu selesai pergerakan ditambah jeda 1 detik
    time.sleep(3)

    # Contoh 2: Bergerak ke pose tertentu
    target_pose_2 = [0.5, -0.5, 1.0, -0.5, 0.5, -1.0, 0.2]
    node.move_to_pose(target_pose_2, 3.0)
    
    time.sleep(4)

    # Menutup Node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
