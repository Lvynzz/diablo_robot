# `diablo_web_interface`

Web HMI Diablo dengan arsitektur:

```text
React/Vite browser <- WebSocket /ws + REST /api/* -> FastAPI web_node.py
                                                       |
                                                     rclpy
                                                       |
                                           ROS 2 / Nav2 / Diablo
```

README lengkap dan deployment ada di
[`diablo_web_interface/README.md`](../../diablo_web_interface/README.md) dan
[`DEPLOY.md`](../../diablo_web_interface/DEPLOY.md).

## Build

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to diablo_web_interface
source install/setup.bash
```

Jika mengubah frontend React:

```bash
cd ~/diablo_ws/src/diablo_web_interface
npm install
npm run build
```

Tanpa Node.js, folder `diablo_web_interface/static/` menyediakan fallback
preview untuk memeriksa layout.

## Launch

Jalankan `full_body_hardware.launch.py use_ekf:=false use_local_odom:=true`
terlebih dahulu agar `/diablo/odometry` dan TF `odom -> diablo_base_link`
tersedia.

Untuk HMI/teleop:

```bash
ros2 launch diablo_web_interface web_interface.launch.py
```

Untuk HMI + odom roda lokal + Nav2:

```bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  map:=/absolute/path/to/map.yaml
```

Default host/port adalah `0.0.0.0:8000`. Buka `http://IP_ROBOT:8000` dari
laptop.

## Panel dan topic utama

- Drive Control: command manual `MotionCtrl`, keybind, STOP, telemetry odom.
- Navigation: map, costmap, initial pose, goal, stations, startup controls.
- ROS Topics: dynamic topic echo terbatas.
- Settings: endpoint dan checklist konfigurasi.

Topic/service default:

```text
/diablo/MotionCmd/manual -> mux -> /diablo/MotionCmd
/diablo/odometry
/scan
/diablo/sensor/Motors
/diablo/reset_pose
/diablo/reset_odom
/diablo/reset_encoder
/start_motor
```

Drive Control terkunci sampai `/diablo/sensor/Motors` memberi feedback. Tombol
**START HARDWARE** menjalankan command Diablo/Dynamixel yang dikonfigurasi dan
memanggil service LiDAR atau menjalankan `lidar_start_command`.

Reset pose tidak terjadi otomatis. Kirim `Bool(data=true)` ke
`/diablo/reset_pose` atau panggil service `/diablo/reset_odom` hanya jika ingin
menjadikan pose saat ini sebagai origin baru.

## Pengaturan hardware

Contoh command LiDAR harus disesuaikan dengan model dan port:

```bash
ros2 launch diablo_web_interface web_interface.launch.py \
  lidar_start_command:="ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p serial_port:=/dev/ttyUSBx -p serial_baudrate:=256000"
```

AMCL, Nav2, dan Mapping menerima command opsional melalui
`localization_start_command`, `navigation_start_command`, dan
`mapping_start_command`. Jangan menjalankan driver yang sama dua kali.
