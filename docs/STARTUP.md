# Startup dan deployment mini PC

Dokumen ini membedakan tiga hal yang sering tertukar:

1. menjalankan Web HMI;
2. menjalankan driver ROS/hardware;
3. menjalankan seluruh stack Nav2.

Web dapat hidup tanpa motor, sedangkan Drive Control hanya dibuka setelah
feedback Diablo diterima. Ini adalah gate keselamatan yang disengaja.

## 1. Persiapan mini PC

```bash
sudo usermod -aG dialout "$USER"
```

Logout/login kembali, lalu cek device serial:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Gunakan path `/dev/serial/by-id/...` bila tersedia karena lebih stabil daripada
`/dev/ttyUSB0`. Port LiDAR dan U2D2 tidak boleh sama.

Source environment:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
```

## 2. Build setelah clone

```bash
cd ~/diablo_ws
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. Mode HMI dengan startup hardware dari tombol

Jalankan Web HMI:

```bash
ros2 launch diablo_web_interface web_interface.launch.py \
  lidar_start_command:="ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p serial_port:=/dev/serial/by-id/LI-DAR-DEVICE \
    -p serial_baudrate:=256000"
```

Parameter LiDAR di atas hanya contoh. Sesuaikan `serial_port`, baudrate, dan
frame dengan model LiDAR yang terpasang.

Pada browser buka `http://IP_MINI_PC:8000`, masuk ke Navigation, lalu tekan
**START HARDWARE**. Backend akan menjalankan command Diablo, Dynamixel, dan
LiDAR yang dikonfigurasi. Tunggu `DIABLO ROS2` menjadi `READY`.

Jika LiDAR sudah dikelola oleh launch/service lain, kosongkan
`lidar_start_command`. Pastikan node LiDAR tersebut sudah menyediakan
`/start_motor` sebelum tombol ditekan.

## 4. Mode driver manual

Mode ini berguna untuk diagnosis. Jangan menjalankannya bersamaan dengan
`START HARDWARE` yang akan menjalankan command yang sama.

Terminal 1 — driver Diablo:

```bash
ros2 run diablo_ctrl diablo_ctrl_node
```

Terminal 2 — controller Dynamixel upper body, bila diperlukan:

```bash
ros2 launch diablo_bringup six_joint_move.launch.py
```

Terminal 3 — LiDAR:

```bash
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/ttyUSBx -p serial_baudrate:=256000
```

Terminal 4 — HMI:

```bash
ros2 launch diablo_web_interface web_interface.launch.py
```

## 5. Mode Nav2

Sebelum menjalankan Nav2, pastikan topic dan TF berikut ada:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser
```

Kemudian:

```bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  map:=/absolute/path/to/map.yaml
```

`nav2_web.launch.py` tidak otomatis mengetahui model LiDAR. Gunakan
`lidar_start_command:=...` atau jalankan driver LiDAR lebih dahulu.

## 6. Autostart web saat boot dengan systemd

Buat unit di mini PC, sesuaikan `User` dan path workspace:

```ini
# /etc/systemd/system/diablo-web.service
[Unit]
Description=Diablo Web HMI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=alvin
WorkingDirectory=/home/alvin/diablo_ws
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/alvin/diablo_ws/install/setup.bash && exec ros2 launch diablo_web_interface web_interface.launch.py'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Aktifkan:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now diablo-web.service
systemctl status diablo-web.service
journalctl -u diablo-web.service -f
```

Unit ini hanya menjalankan HMI dan mux. Operator tetap harus menekan
**START HARDWARE**. Membuat service yang langsung memberi torque atau command
gerak otomatis memerlukan prosedur keselamatan tambahan.

## 7. Masalah umum

| Gejala | Pemeriksaan |
| --- | --- |
| HMI tidak terbuka | `systemctl status`, port `8000`, dan firewall |
| `DIABLO ROS2` error | port `/dev/ttyS3`, power, dan log `diablo_ctrl_node` |
| U2D2 gagal | port/baudrate di xacro dan akses `dialout` |
| `/start_motor` unavailable | node `sllidar_node` belum berjalan |
| `/scan` kosong | model, baudrate, serial port, dan permission LiDAR |
| Drive Control terkunci | belum ada message `/diablo/sensor/Motors` |
| Nav2 tidak aktif | cek `/scan`, `/odom`, TF, map, dan lifecycle nodes |
