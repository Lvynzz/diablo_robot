#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty

# Pengaturan tombol keyboard untuk teleop
# Format: 'tombol': (index_joint, perubahan_sudut)
# Misalnya: 'q' menambah sudut joint 0 (joint1), 'a' menguranginya.
key_bindings = {
    'q': (0, 0.1), 'a': (0, -0.1), # joint1 (Bahu Kanan)
    'w': (1, 0.1), 's': (1, -0.1), # joint2 (Lengan Atas Kanan)
    'e': (2, 0.1), 'd': (2, -0.1), # joint3 (Sikut Kanan)
    'r': (3, 0.1), 'f': (3, -0.1), # joint6 (Bahu Kiri)
    't': (4, 0.1), 'g': (4, -0.1), # joint7 (Lengan Atas Kiri)
    'y': (5, 0.1), 'h': (5, -0.1), # joint8 (Sikut Kiri)
    'u': (6, 0.1), 'j': (6, -0.1), # joint11 (Leher)
}

def get_key():
    """Fungsi pembantu untuk membaca tombol keyboard yang ditekan."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class JointTrajectoryTeleop(Node):
    def __init__(self):
        super().__init__('seven_joint_teleop_node')
        
        # Publisher ke topik /seven_joint_trajectory_controller/joint_trajectory
        # Topik ini menerima pesan berupa rentetan titik yang harus dilalui joint
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/seven_joint_trajectory_controller/joint_trajectory', 
            10)
            
        # Nama-nama joint yang akan dikontrol
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint6', 'joint7', 'joint8', 'joint11'
        ]
        
        # Posisi saat ini, kita mulai dari posisi 0 (bisa disesuaikan dengan resting position)
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        print("Mulai Teleop. Tekan Ctrl+C untuk keluar.")
        print("Gunakan Q/A, W/S, E/D, R/F, T/G, Y/H, U/J untuk mengontrol masing-masing joint.")

    def publish_positions(self):
        # Membuat objek pesan JointTrajectory
        msg = JointTrajectory()
        # Mengisi nama joint agar controller tahu posisi mana untuk joint yang mana
        msg.joint_names = self.joint_names
        
        # Membuat satu titik pergerakan (point)
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        
        # Waktu dari awal eksekusi hingga titik ini tercapai.
        # Karena ini teleop (langsung bergerak), kita beri waktu 0.2 detik agar tidak terlalu kasar.
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 0.2 detik
        
        # Menambahkan titik ke pesan trajectory
        msg.points.append(point)
        
        # Publikasikan ke controller
        self.publisher_.publish(msg)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    teleop_node = JointTrajectoryTeleop()

    try:
        while rclpy.ok():
            key = get_key()
            if key == '\x03': # Ctrl+C
                break
                
            if key in key_bindings:
                joint_idx, step = key_bindings[key]
                # Update posisi joint yang sesuai
                teleop_node.current_positions[joint_idx] += step
                # Print untuk info ke terminal
                print(f"Update: {teleop_node.joint_names[joint_idx]} = {teleop_node.current_positions[joint_idx]:.2f}")
                
                # Kirim perintah pergerakan
                teleop_node.publish_positions()
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
