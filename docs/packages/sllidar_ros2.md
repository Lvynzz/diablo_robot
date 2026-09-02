# `sllidar_ros2`

Driver ROS 2 untuk SLAMTEC/RPLIDAR. Package ini berada sebagai submodule
`sllidar_ros2`; README upstream tetap tersedia di
[`sllidar_ros2/README.md`](../../sllidar_ros2/README.md).

## Build

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2
source install/setup.bash
```

Pastikan user memiliki akses serial:

```bash
sudo usermod -aG dialout "$USER"
ls -l /dev/serial/by-id/
```

Logout/login setelah menambah grup `dialout`.

## Menjalankan

Pilih launch sesuai model LiDAR dan sesuaikan device/baudrate:

```bash
ros2 launch sllidar_ros2 sllidar_a2m7_launch.py \
  serial_port:=/dev/ttyUSBx
```

Untuk model lain, lihat daftar `sllidar_*_launch.py` di folder `launch/`.
Jangan menebak model atau baudrate. Verifikasi dengan label perangkat dan
manual hardware.

Cek topic dan service:

```bash
ros2 topic echo /scan --once
ros2 topic info /scan
ros2 service list | grep -E 'start_motor|stop_motor'
```

Node ini menyediakan `/start_motor` dan `/stop_motor` bertipe
`std_srvs/srv/Empty` pada versi yang dipakai workspace ini. Web HMI memakai
`/start_motor` sebagai default jika `lidar_start_command` tidak diisi.

## Dengan Web HMI

Ada dua pilihan:

1. Jalankan LiDAR lebih dahulu, lalu jalankan Web HMI. Tombol **START HARDWARE**
   akan memanggil `/start_motor`.
2. Isi `lidar_start_command` pada launch Web HMI sehingga backend menjalankan
   `sllidar_node` sebagai child process.

Jangan menjalankan dua node LiDAR pada serial port yang sama. Untuk Nav2,
pastikan juga ada TF dari frame LiDAR ke `base_link` dan QoS `/scan` sesuai
konfigurasi costmap.
