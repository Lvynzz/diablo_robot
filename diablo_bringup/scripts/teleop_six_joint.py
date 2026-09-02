#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading
import math
import os
import re

class TeleopSixJoint(Node):
    def __init__(self):
        super().__init__('teleop_six_joint')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/six_joint_position_controller/commands', 10)
        
        # Mapping ID ke index array dalam Float64MultiArray (berurutan: 1, 2, 3, 6, 7, 8)
        self.id_to_index = {1: 0, 2: 1, 3: 2, 6: 3, 7: 4, 8: 5}
        self.current_degrees = [0.0] * 6
        
        # Ekstrak nilai resting position dari file
        self.load_resting_positions()
        
        # Beri jeda sejenak agar publisher ROS 2 terhubung, lalu kirim posisi rest
        # agar robot tidak tersentak/melompat secara tiba-tiba
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.initial_publish)
        self.initial_published = False

    def initial_publish(self):
        if not self.initial_published:
            self.publish_positions()
            self.initial_published = True
            # Hentikan timer setelah dipublish sekali
            self.timer.cancel()

    def load_resting_positions(self):
        # Cari file di berbagai kemungkinan workspace
        possible_paths = [
            os.path.expanduser('~/alvin_ws/src/diablo_bringup/ID_dynamixel'),
            os.path.expanduser('~/diablo_ws/src/diablo_bringup/ID_dynamixel'),
            '/home/alvin/diablo_ws/src/diablo_bringup/ID_dynamixel'
        ]
        
        filepath = None
        for p in possible_paths:
            if os.path.exists(p):
                filepath = p
                break
                
        if not filepath:
            print("[!] File ID_dynamixel tidak ditemukan. Menggunakan 0.0 sebagai default.")
            return

        print(f"\n[*] Membaca resting position dari {filepath} ...")
        try:
            with open(filepath, 'r') as f:
                content = f.read()
            
            # Parsing teks sederhana untuk mencari ID dan derajatnya
            current_id = None
            for line in content.split('\n'):
                line = line.strip()
                if line.startswith('ID'):
                    match = re.search(r'ID(\d+):', line)
                    if match:
                        current_id = int(match.group(1))
                elif line.startswith('Resting position value:'):
                    if current_id in self.id_to_index:
                        val_str = line.split(':')[1].strip()
                        self.current_degrees[self.id_to_index[current_id]] = float(val_str)
                        
            print("[*] Berhasil menetapkan Resting Positions:")
            for dxl_id, idx in self.id_to_index.items():
                print(f"    - ID {dxl_id}  -> {self.current_degrees[idx]} derajat")
                
        except Exception as e:
            print(f"[!] Gagal membaca file: {e}. Menggunakan 0.0 sebagai default.")

    def publish_positions(self):
        msg = Float64MultiArray()
        # Konversi ke radian karena ROS 2 mengontrol posisi dalam radian
        msg.data = [math.radians(deg) for deg in self.current_degrees]
        self.publisher_.publish(msg)

def ui_loop(node):
    print("\n" + "="*60)
    print("🤖 TELEOPERASI SIX JOINT DYNAMIXEL 🤖")
    print("="*60)
    print("ℹ️  Standby pada Resting Position untuk mencegah lompatan.")
    print("📝 Format Input: [ID] [Derajat] [ID] [Derajat] ...")
    print("💡 Contoh Input: 1 225 3 130")
    print("   (Artinya -> Joint ID 1 ke 225°, Joint ID 3 ke 130°)")
    print("⛔ Ketik 'quit' atau 'exit' untuk keluar dari program.")
    print("="*60 + "\n")
    
    while rclpy.ok():
        try:
            user_input = input(">> Masukkan perintah: ").strip()
            if user_input.lower() in ['quit', 'exit']:
                print("Menutup program teleoperasi...")
                break
            if not user_input:
                continue
                
            parts = user_input.split()
            if len(parts) % 2 != 0:
                print("[!] Format salah. Pastikan format berpasangan (contoh: 1 225).")
                continue
                
            valid_update = False
            for i in range(0, len(parts), 2):
                dxl_id = int(parts[i])
                deg_val = float(parts[i+1])
                
                if dxl_id in node.id_to_index:
                    node.current_degrees[node.id_to_index[dxl_id]] = deg_val
                    valid_update = True
                else:
                    print(f"[!] ID {dxl_id} tidak valid (Gunakan ID: 1, 2, 3, 6, 7, 8).")
            
            if valid_update:
                node.publish_positions()
                print(f"[+] Berhasil! Status derajat saat ini:")
                status = " | ".join([f"ID{id}:{node.current_degrees[idx]}°" for id, idx in node.id_to_index.items()])
                print(f"    {status}\n")
                
        except ValueError:
            print("[!] Harap masukkan angka yang valid!\n")
        except Exception as e:
            print(f"[!] Terjadi error: {e}\n")
            
    # Matikan node dengan aman
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSixJoint()
    
    # Jalankan listener ROS 2 di background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Jalankan UI interaktif di foreground
    try:
        ui_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
