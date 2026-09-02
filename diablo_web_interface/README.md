# diablo_web_interface

Package Nav2 dan web dashboard untuk robot Diablo. Package ini dibuat di
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
sidebar pilihan panel di kiri, tiga readout posisi dari wheel odometry, quick
actions, Motion Control, wheel encoder telemetry, keybind legend, dan trajectory
map. Panel obstacle laser depan serta magnetic navigation sensor sengaja tidak
ditampilkan pada versi ini. Setiap panel dapat ditutup dari tombol chevron di
header; sidebar juga dapat diciutkan sehingga ikon tetap bisa dipakai untuk
berpindah panel.

Panel Navigation mengikuti layout HMI referensi: Navigation Map (map PGM dari
`/map` dengan overlay global/local costmap dan inflation layer), Pose & Stations,
Navigation Controls, serta Navigation Log History. Pemilihan map saat ini
memilih asset dan memberi indikasi bahwa map_server perlu direstart untuk
menerapkannya; startup localization/Nav2/mapping disediakan sebagai command
yang dapat dikonfigurasi.

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
| `web_node` | FastAPI + WebSocket: teleop, goal Nav2, initial pose, telemetry, topic echo, hardware startup, reset odom/encoder, start LiDAR |
| `motion_cmd_bridge` | `Twist` Nav2 → `MotionCtrl` Diablo |
| `motion_cmd_mux` | Pemilih manual/auto dengan command watchdog |
| `wheel_odom` | Estimasi `/odom` dari `motion_msgs/LegMotors` dan TF `odom → base_link` |
| `navigation.launch.py` | map server, AMCL, costmap, planner, controller dan lifecycle Nav2 |
| `mapping.launch.py` | SLAM Toolbox + odometri roda opsional |
| `nav2_web.launch.py` | Launch gabungan web, mux, wheel odom dan Nav2 |

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

## Menjalankan web + Nav2

Jalankan launch berikut pada mesin ROS. Tombol **START HARDWARE** di panel
Navigation akan menjalankan driver Diablo dan Dynamixel yang dikonfigurasi;
LiDAR bisa dijalankan lewat command atau service start. Jika driver sudah
dijalankan oleh systemd/launch lain, command terkait dapat dikosongkan dan HMI
tetap menunggu feedback ROS. Kemudian:

```bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  map:=/absolute/path/to/my_map.yaml \
  lidar_start_command:='ros2 launch <lidar_package> <lidar_launch>.launch.py'
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

### START HARDWARE

Tombol **START HARDWARE** menjalankan command yang didefinisikan saat launch,
lalu menunggu feedback ROS nyata sebelum membuka Drive Control:

- Diablo ROS2: default `ros2 run diablo_ctrl diablo_ctrl_node`.
- Dynamixel U2D2: default `ros2 launch diablo_bringup six_joint_move.launch.py`.
- LiDAR: driver `sllidar_ros2` dari `amr_ws` memakai service
  `std_srvs/srv/Empty` pada `/start_motor`; backend mencoba endpoint itu lebih
  dulu dan juga mendukung `std_srvs/srv/Trigger` bila `lidar_start_service:=...`
  diarahkan ke service custom. Bila node LiDAR belum dijalankan, isi
  `lidar_start_command:=...` sesuai model dan port LiDAR robot.

Status `READY` untuk gerak manual baru aktif setelah message
`/diablo/sensor/Motors` diterima. Log proses startup disimpan di
`/tmp/diablo_web_interface-{diablo,lidar,dynamixel}.log`. Launch Dynamixel yang
ada memakai port `/dev/ttyUSB0` di `diablo_bringup/urdf/six_joint.urdf.xacro`;
ubah port itu jika nama device U2D2 di robot berbeda.

Tombol AMCL, Nav2 dan Mapping memakai command opsional berikut. Karena setup
Nav2 belum final, default-nya kosong dan tombol hanya mencatat bahwa command
belum dikonfigurasi:

```bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  localization_start_command:='ros2 launch <package> <amcl_launch>.launch.py' \
  navigation_start_command:='ros2 launch <package> <nav2_launch>.launch.py' \
  mapping_start_command:='ros2 launch <package> <slam_launch>.launch.py'
```

## Prasyarat Nav2 yang perlu tersedia di robot

Implementasi Diablo yang ada saat ini menyediakan IMU, battery, body state dan
motor telemetry. Nav2 masih membutuhkan:

1. `sensor_msgs/LaserScan` pada `/scan` (atau set `scan_topic:=...`) dan TF
   dari frame laser ke frame robot.
2. TF `map → odom` dari AMCL atau SLAM Toolbox. Launch standar menyediakan
   AMCL; gunakan `mapping.launch.py` bila ingin membuat peta.
3. Frame robot yang konsisten. Konfigurasi default memakai `base_link`. URDF
   full-body yang ada memakai `diablo_base_link`, sehingga jalankan dengan
   `base_frame:=diablo_base_link` dan pastikan sensor/Nav2 memakai params yang
   sama.

`wheel_odom` menggunakan `left_wheel_pos/right_wheel_pos` dalam radian dan
revolution counter dari `LegMotors`. Nilai awalnya `wheel_radius=0.105`,
`track_width=0.3751`, arah kiri `+1` dan kanan `-1` mengikuti konstanta SDK.
Kalibrasikan di tempat sebelum navigasi: bila maju menghasilkan odom mundur,
ubah `left_wheel_direction`/`right_wheel_direction`; bila jarak tidak sesuai,
ubah radius. Untuk odom eksternal, jalankan `enable_wheel_odom:=false` dan
gunakan `odom_topic:=/nama_odom`.

Shortcut teleoperasi default mengikuti [dokumentasi resmi Diablo](https://github.com/DDTRobot/diablo_ros2/blob/main/docs/docs_en/README_EN.md):
`W/S` maju-mundur, `A/D` putar, `Q/E` roll, `Z` standing mode, `X` crawling
mode, serta kontrol tinggi/pitch. Keybind dapat di-remap dari panel **Keybind
Legend** dan tersimpan di browser.

Map fallback `maps/empty.yaml` hanya untuk memastikan launch dapat dimulai;
map itu bukan peta lingkungan nyata dan tidak boleh dipakai untuk navigasi
robot di lapangan.

## Alur kontrol

Web mengirim command manual berkala. Mux menghentikan output ketika command
terpilih diam lebih dari sekitar 0.35 detik. Saat goal dikirim, web mengganti
mode ke `auto`; setelah goal selesai/dibatalkan, mode kembali `manual`.
Tombol `STOP` mengirim command nol dan memaksa mode manual.

Topic echo memakai dynamic ROS subscription maksimal empat topic dan membatasi
payload tiap message supaya tidak membebani WebSocket.
