# diablo_web_interface

Package web dashboard untuk hardware, mapping SLAM dan teleoperasi robot Diablo.
Navigasi Nav2 belum menjadi bagian dari workflow web ini. Package ini dibuat di
`diablo_ws/src` dengan pola yang sama seperti `amr_web_interface`, tetapi
perintah teleoperasinya memakai message asli Diablo:

```text
motion_msgs/MotionCtrl
  /diablo/MotionCmd/manual  ->  motion_cmd_mux  ->  /diablo/MotionCmd
```

`diablo_ctrl_node` tetap menjadi node yang berbicara ke SDK/serial Diablo.
Nav2 menghasilkan `geometry_msgs/Twist`; `motion_cmd_bridge` mengubahnya ke
`MotionCtrl` lalu mux memilih sumber manual atau otomatis.

## Web HMI Architecture

```text
Browser (React + TypeScript + Vite)
        ↕ WebSocket /ws + REST /api/*
FastAPI (web_node.py) ↔ rclpy ↔ ROS 2 / Nav2 ↔ Diablo driver
```

Frontend React/Vite berada langsung di `src/` pada package ini. Backend
FastAPI berada di `diablo_web_interface/`. Source frontend dipakai untuk
development dan menghasilkan `dist/` untuk deployment; bila `dist/` belum
dibuat, backend memakai frontend fallback di
  `diablo_web_interface/static/`.

Panel Drive Control menggunakan tema HMI industrial berwarna biru-abu terang,
sidebar pilihan panel di kiri, tiga readout posisi dari resettable wheel odometry, quick
actions, Motion Control, wheel encoder telemetry, keybind legend, dan trajectory
map. Panel obstacle laser depan serta magnetic navigation sensor sengaja tidak
ditampilkan pada versi ini. Setiap panel dapat ditutup dari tombol chevron di
header; sidebar juga dapat diciutkan sehingga ikon tetap bisa dipakai untuk
berpindah panel.

Panel Mapping menampilkan occupancy grid live dari `/map`, status tiga komponen
hardware, kontrol start/stop SLAM Toolbox, teleoperasi W/A/S/D, dan penyimpanan
pasangan file `.pgm` + `.yaml`. Tombol mapping hanya dibuka setelah feedback
motor Diablo, LiDAR dan Dynamixel diterima.

Panel Navigation memakai tombol lifecycle ON/OFF untuk hardware, localization,
navigation dan mapping. Saat komponen yang sedang aktif ditekan kembali, HMI
meminta konfirmasi **IYA, STOP** atau **TIDAK**. Hardware OFF lebih dulu
menghentikan mapping/navigation/localization yang dijalankan oleh HMI, mengirim
command berhenti, lalu menghentikan process group hardware yang dibuat oleh HMI.
Process yang dijalankan dari terminal atau systemd sengaja tidak dibunuh oleh
tombol ini.

Panel Drive juga menyediakan slider posisi Dynamixel ID 1–10. Slider dimulai di
tengah, tidak mengirim command saat halaman dibuka, dan mengirim satu target
`trajectory_msgs/JointTrajectory` ketika dilepas setelah `/joint_states` tersedia.
ID 11–12 tidak ditampilkan karena dipakai untuk human detection.

Untuk melihat HMI dari laptop:

```bash
cd ~/diablo_ws/src/diablo_web_interface
npm install
VITE_ROBOT_URL=http://<IP-ROBOT>:8000 npm run dev
```

Buka `http://localhost:3000`. Tanpa koneksi robot, halaman React tetap
menampilkan data demo lokal sehingga layout Drive Control, Navigation, ROS
Topics, dan Settings dapat diperiksa.

## Isi package

| Bagian | Fungsi |
| --- | --- |
| `web_node` | FastAPI + WebSocket: hardware startup, SLAM mapping, teleop, save map, telemetry, topic echo dan reset odom/encoder |
| `motion_cmd_bridge` | `Twist` Nav2 → `MotionCtrl` Diablo |
| `motion_cmd_mux` | Pemilih manual/auto dengan command watchdog |
| `wheel_odom` | Estimasi odom encoder legacy untuk launch yang tidak memakai ros2_control |
| `diablo_localization` | Odom roda lokal resettable pada `/diablo/odometry`; EKF tetap opsional |
| `navigation.launch.py` | map server, AMCL, costmap, planner, controller dan lifecycle Nav2 |
| `mapping.launch.py` | SLAM Toolbox + odometri roda opsional |
| `nav2_web.launch.py` | Launch gabungan web, mux, odom roda lokal dan Nav2 |

## Build di laptop/robot

Jalankan di mesin yang memiliki ROS 2 Humble, Nav2, FastAPI dan Uvicorn:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to diablo_web_interface
source install/setup.bash
```

Laptop saat package ini dibuat belum memiliki dependency Nav2/FastAPI, jadi
build runtime penuh perlu dilakukan setelah source dipindahkan ke robot atau
dependency dipasang di laptop.

## Menjalankan web + mapping

Jalankan web interface pada mesin ROS. Tombol **ON HARDWARE** akan menjalankan
driver Diablo, LiDAR dan Dynamixel dengan port udev robot, lalu menunggu semua
feedback sebelum membuka mapping dan teleoperasi:

```bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_web_interface web_interface.launch.py
```

Buka `http://IP_MESIN_ROS:8000`. Port bisa diganti dengan
`port:=8080`. Untuk uji teleoperasi saja:

```bash
ros2 launch diablo_web_interface web_interface.launch.py
```

Untuk menjalankan backend dari source setelah dependency Python/ROS tersedia:

```bash
cd ~/diablo_ws/src/diablo_web_interface
source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m diablo_web_interface.web_node
```

Buka `http://IP_MESIN_ROS:8000`.

`diablo_ctrl_node` harus mendengarkan `/diablo/MotionCmd`. Jika juga ingin
memakai keyboard teleop lama bersamaan dengan mux, remap publisher lamanya:

```bash
ros2 run diablo_teleop teleop_node \
  --ros-args -r diablo/MotionCmd:=/diablo/MotionCmd/manual
```

Jangan menjalankan dua mux yang sama-sama mem-publish ke
`/diablo/MotionCmd`.

### ON HARDWARE

Tombol **ON HARDWARE** menjalankan command yang didefinisikan saat launch dan
memantau tiga feedback ROS nyata:

- Diablo ROS2: `ros2 run diablo_ctrl diablo_ctrl_node --ros-args -p controller_port:=/dev/diablo_controller`.
- LiDAR: `ros2 launch sllidar_ros2 sllidar_a2m7_launch.py serial_port:=/dev/rplidar frame_id:=laser`.
- Dynamixel: mode upper-body `full_body_hardware.launch.py` dengan `/dev/u2d2_arm`, `/dev/u2d2_hand`, dan baudrate `1000000`.

Gate mapping aktif setelah `/diablo/sensor/Motors` dan `/scan` diterima;
Dynamixel dilaporkan terpisah dan tidak lagi memblokir SLAM. Status **ALL
READY** tetap berarti ketiga feedback, termasuk joint lengan, telah diterima.
Odometri roda standalone dijalankan oleh web launch sehingga kegagalan U2D2
tidak menghilangkan `/diablo/odometry`. Log startup disimpan di
`/tmp/diablo_web_interface-{diablo,lidar,dynamixel}.log`.
Launch yang sama menerbitkan TF statis `diablo_base_link → laser` pada pose
LiDAR default `(x=0, y=0.08, z=0.17, yaw=π)`, sehingga SLAM tidak menunggu TF
yang hanya tersedia saat full-body hardware aktif. Pose dapat dikalibrasi
melalui argumen `lidar_x`, `lidar_y`, `lidar_z`, dan `lidar_yaw`.

### START MAPPING dan SAVE MAP

```bash
ros2 launch diablo_web_interface mapping.launch.py \
  enable_wheel_odom:=false scan_topic:=/scan
```

Tombol **START MAPPING** menjalankan command tersebut dengan occupancy grid
default dari `config/slam_toolbox.yaml`. Setelah area selesai dipindai,
masukkan nama map lalu klik **SAVE**. Backend menjalankan
`nav2_map_server map_saver_cli` dan menulis pasangan `<nama>.pgm` serta
`<nama>.yaml` ke `diablo_bringup/map/`. Nama yang sudah ada tidak ditimpa.

## Prasyarat mapping yang perlu tersedia di robot

Implementasi Diablo yang ada saat ini menyediakan IMU, battery, body state dan
motor telemetry. SLAM Toolbox membutuhkan:

1. `sensor_msgs/LaserScan` pada `/scan` (atau set `scan_topic:=...`) dan TF
   dari frame laser ke frame robot.
2. TF `odom → diablo_base_link` dari local wheel odometry. Jalankan full-body
   dengan `use_ekf:=false use_local_odom:=true`.
3. Frame robot yang konsisten. Konfigurasi default memakai `diablo_base_link`.
   IMU driver memakai `diablo_robot`; static TF IMU hanya diperlukan bila
   EKF eksperimental diaktifkan. Ubah transform jika pemasangan sensor tidak
   sejajar.

4. TF `map → odom` akan diterbitkan SLAM Toolbox selama mapping.

`wheel_odom` dan adapter base menggunakan `left_wheel_pos/right_wheel_pos`
dalam radian serta revolution counter dari `LegMotors`. Nilai awalnya
`wheel_radius=0.093`, `track_width=0.475`, arah kiri `+1` dan kanan `+1`
mengikuti konstanta SDK. Kalibrasikan di tempat sebelum navigasi: bila maju
menghasilkan odom mundur, ubah `left_feedback_sign`/`right_feedback_sign` pada
full-body launch; bila jarak tidak sesuai, ubah radius. Dalam mode default,
gunakan `/diablo/odometry`; odom mentah tetap ada di
`/diablo_base_controller/odom`. Jangan menjalankan `wheel_odom` bersamaan
dengan local odom full-body karena keduanya dapat mempublikasikan odometry/TF
yang bersaing.

Standalone `wheel_odom` mengabaikan lonjakan satu sampel di atas 1.5 radian
(`max_wheel_delta`) dan menjadikannya baseline baru. Filter ini mencegah nilai
revolution counter yang belum stabil saat serial driver baru hidup menggeser
pose beberapa meter.

Reset pose lokal hanya dengan sengaja. Opsi `-w 1` menunggu node reset
terhubung sebelum mengirim pesan one-shot:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

Tanpa perintah tersebut, pose odom lokal tetap berlanjut selama node tidak
direstart.

Teleoperasi mapping memakai `W/S` untuk maju-mundur dan `A/D` untuk berputar.
Command dikirim berkala selama tombol ditahan dan otomatis dihentikan saat
tombol dilepas atau halaman kehilangan fokus.

Map fallback `maps/empty.yaml` bukan peta lingkungan nyata. Hasil mapping yang
disimpan berada di `diablo_bringup/map` dan tidak dibuat oleh UI navigasi.

## Alur kontrol

Web mengirim command manual berkala. Mux menghentikan output ketika command
terpilih diam lebih dari sekitar 0.35 detik. Tombol `STOP` mengirim command
nol dan memaksa mode manual. Mapping hanya dapat dimulai setelah seluruh gate
hardware siap.

Topic echo memakai dynamic ROS subscription maksimal empat topic dan membatasi
payload tiap message supaya tidak membebani WebSocket.
