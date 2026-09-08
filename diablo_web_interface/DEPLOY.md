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

- **Mapping** — occupancy grid `/map`, gate hardware untuk Diablo/LiDAR/Dynamixel,
  start/stop SLAM Toolbox, teleoperasi W/A/S/D, dan save PGM/YAML.
- **ROS Topics** — catalog topic, filter, dynamic echo sampai empat topic,
  dan payload JSON yang dibatasi ukuran oleh backend.
- **Settings** — routing command Diablo, endpoint WebSocket/REST, frame map,
  serta checklist deployment.

Teleoperasi mapping memakai `W/S` maju-mundur dan `A/D` putar. Command dikirim
berkala selama tombol ditahan dan dihentikan saat tombol dilepas atau halaman
kehilangan fokus.

Jika menggunakan source web statis tanpa Node.js, folder
`diablo_web_interface/static/` menyediakan fallback preview sederhana. Untuk
HMI production gunakan `npm run build` terlebih dahulu.

## Hardware startup gate

Klik **ON HARDWARE** dari panel Mapping. Backend menjalankan command yang
dikonfigurasi dan baru mengizinkan mapping/teleop setelah feedback motor,
LiDAR dan Dynamixel diterima. Default command:

```text
Diablo ROS2   : ros2 run diablo_ctrl diablo_ctrl_node --ros-args -p controller_port:=/dev/diablo_controller
LiDAR         : ros2 launch sllidar_ros2 sllidar_a2m7_launch.py serial_port:=/dev/rplidar frame_id:=laser
Dynamixel     : full_body_hardware.launch.py dengan /dev/u2d2_arm dan /dev/u2d2_hand
```

Mapping dijalankan dengan default:

```bash
ros2 launch diablo_web_interface mapping.launch.py \
  enable_wheel_odom:=false scan_topic:=/scan
```

Setelah selesai, tombol **SAVE MAP** menjalankan `nav2_map_server map_saver_cli`
dan membuat `<name>.pgm` + `<name>.yaml` di `diablo_bringup/map/`. File yang
sudah ada tidak ditimpa. Log child process ada di
`/tmp/diablo_web_interface-*.log`.
