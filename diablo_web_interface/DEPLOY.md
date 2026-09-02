# Diablo Web HMI — Deployment

Package ini memakai arsitektur yang sama dengan HMI AMR:

```text
Browser (React + TypeScript + Vite) ←→ WebSocket /ws
    ↕ REST /api/*
FastAPI (web_node.py) ←→ rclpy ←→ ROS 2 / Nav2 ←→ Diablo driver
```

Frontend source ada di `src/`. Hasil production build dibuat Vite di
`dist/`, kemudian FastAPI akan memilih folder itu saat dijalankan dari source.

## Development mode

Di laptop operator/developer:

```bash
cd ~/diablo_ws/src/diablo_web_interface
npm install
VITE_ROBOT_URL=http://<IP-ROBOT>:8000 npm run dev
```

Buka `http://localhost:3000`. Vite meneruskan `/api/*` dan `/ws` ke FastAPI
robot. Jika robot belum tersedia, HMI menampilkan local preview data agar
layout dan panel tetap dapat diperiksa.

## Production build

```bash
cd ~/diablo_ws/src/diablo_web_interface
npm install
npm run build
```

Build menghasilkan `dist/index.html` dan `dist/assets/*`. Saat build package
ROS, `setup.py` otomatis mengambil `dist/` dan menginstalnya sebagai
`share/diablo_web_interface/static/`.

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to diablo_web_interface
source install/setup.bash
ros2 launch diablo_web_interface nav2_web.launch.py map:=/path/to/map.yaml
```

## FastAPI-only source run

Untuk menjalankan backend dari source setelah dependency Python terpasang:

```bash
cd ~/diablo_ws/src/diablo_web_interface
source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m diablo_web_interface.web_node
```

Buka `http://<IP-ROBOT>:8000`.

## HMI panels

- **Drive Control** — layout HMI industrial ringan dengan header `DIABLO ROBOT`,
  sidebar panel di kiri, X/Y/heading dari `/odom` wheel odometry, RESET ODOM,
  RESET ENCODER, START LIDAR, MotionCtrl manual, max speed, body/pitch control,
  keybind legend, wheel feedback, dan trajectory map. Panel front obstacle laser
  dan magnetic navigation sensor tidak ditampilkan.
- **Navigation** — map PGM/`/map`, overlay global costmap, local costmap dan
  inflation layer, form pose/station, goal/initial pose, Navigation Controls
  untuk hardware/AMCL/Nav2/mapping, serta navigation log history.
- **ROS Topics** — catalog topic, filter, dynamic echo sampai empat topic,
  dan payload JSON yang dibatasi ukuran oleh backend.
- **Settings** — routing command Diablo, endpoint WebSocket/REST, frame map,
  serta checklist deployment.

Shortcut default mengikuti dokumentasi resmi Diablo: `W/S` maju-mundur,
`A/D` putar, `Q/E` roll, `Z` standing mode, `X` crawling mode, dan shortcut
postur lainnya. Keycap di panel **Keybind Legend** dapat diklik lalu ditekan
key baru; hasil remap disimpan di browser operator.

Quick action memakai service ROS berikut:

- `RESET ODOM` → `/diablo/reset_odom` (`std_srvs/srv/Trigger`, disediakan node
  `wheel_odom`).
- `RESET ENCODER` → `/diablo/reset_encoder` secara default; dapat diubah dengan
  `reset_encoder_service:=...`.
- `START LIDAR` → `/start_motor` (`std_srvs/Empty`) secara default, mengikuti
  driver `sllidar_ros2` dari `amr_ws`; backend juga mencoba `Trigger` pada
  endpoint yang sama agar driver custom dapat dipakai. Set
  `lidar_start_service:=...` bila nama servicenya berbeda.

Jika menggunakan source web statis tanpa Node.js, folder
`diablo_web_interface/static/` menyediakan fallback preview sederhana. Untuk
HMI production gunakan `npm run build` terlebih dahulu.

## Hardware startup gate

Klik **START HARDWARE** dari Navigation Controls. Backend menjalankan command
yang dikonfigurasi dan baru mengizinkan teleop setelah feedback
`/diablo/sensor/Motors` diterima. Default command:

```text
Diablo ROS2   : ros2 run diablo_ctrl diablo_ctrl_node
Dynamixel     : ros2 launch diablo_bringup six_joint_move.launch.py
LiDAR         : service /start_motor atau lidar_start_command yang diisi operator
```

Contoh launch di robot:

```bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  lidar_start_command:='ros2 launch <lidar_package> <lidar_launch>.launch.py'
```

Command AMCL/Nav2/SLAM dapat diisi melalui `localization_start_command`,
`navigation_start_command`, dan `mapping_start_command`. Bila kosong, tombolnya
tetap tersedia untuk layout tetapi backend mengembalikan status belum
dikonfigurasi. Log child process ada di `/tmp/diablo_web_interface-*.log`.
