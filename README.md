# Diablo ROS 2 Workspace

Workspace ROS 2 untuk robot Diablo: driver utama Diablo, interface Dynamixel,
model full-body, MoveIt 2, LiDAR SLLIDAR, dan Web HMI untuk teleoperasi serta
persiapan navigasi Nav2.

> Status: alur web dan simulasi sudah disiapkan. Pengujian hardware nyata tetap
> harus dilakukan di mini PC robot dengan port serial, model motor, dan model
> LiDAR yang benar.

## Isi workspace

| Bagian | Fungsi |
| --- | --- |
| `diablo_ros2` | SDK/driver Diablo, telemetry, dan message utama |
| `diablo_bringup` | URDF ros2_control dan launch controller Dynamixel |
| `diablo_web_interface` | React/FastAPI HMI, teleop, topic echo, dan Nav2 bridge |
| `diablo_localization` | Odom roda lokal resettable dan EKF opsional |
| `sllidar_ros2` | Driver SLAMTEC/RPLIDAR dan service motor LiDAR |
| `dynamixel_hardware_interface` | Plugin ros2_control untuk U2D2/Dynamixel |
| `dynamixel_interfaces` | Message/service tambahan untuk Dynamixel |
| `diablo_full_body_description` | URDF/xacro full-body untuk simulasi |
| `diablo_full_body_moveit_config` | Konfigurasi MoveIt 2 full-body |
| `diablo_moveit_bridge` | Bridge target pose ke IK/MoveIt |
| `DynamixelSDK` | SDK ROBOTIS dan contoh ROS 2 |

Panduan per package ada di [docs/PACKAGES.md](docs/PACKAGES.md).

## Clone dan submodule

Repository ini memakai submodule. Clone baru:

```bash
git clone --recurse-submodules https://github.com/Lvynzz/diablo_robot.git
cd diablo_robot
```

Jika repository sudah terlanjur di-clone:

```bash
git submodule update --init --recursive
```

Submodule yang diperlukan adalah `diablo_ros2`, `DynamixelSDK`,
`dynamixel_hardware_interface`, dan `sllidar_ros2`.

## Prasyarat

Contoh di bawah menggunakan ROS 2 Humble dan Ubuntu. Install dependency ROS
sesuai package yang ingin dijalankan. Untuk seluruh workspace:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-moveit \
ros-humble-robot-localization
```

Dependency Python Web HMI:

```bash
sudo apt install -y python3-fastapi python3-uvicorn
```

Node.js/npm hanya diperlukan jika ingin mengubah atau membuat production build
frontend React. Fallback static HMI sudah tersedia di package web.

## Build

Jalankan dari root workspace, bukan dari `src`:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build subset untuk mengecek web HMI:

```bash
colcon build --symlink-install --packages-up-to diablo_web_interface
source install/setup.bash
```

Cek package yang dikenali:

```bash
colcon list --names-only
ros2 pkg list | grep -E 'diablo|dynamixel|sllidar|motion_msgs|ception_msgs'
```

## Profil menjalankan robot

### Profil A — Web HMI dengan tombol START HARDWARE

Profil ini menjalankan Web HMI dan membiarkan operator menekan
**START HARDWARE** setelah robot siap secara mekanis.

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_web_interface web_interface.launch.py
```

Secara default tombol tersebut menjalankan:

```text
Diablo    : ros2 run diablo_ctrl diablo_ctrl_node
Dynamixel : ros2 launch diablo_bringup six_joint_move.launch.py
LiDAR     : memanggil service /start_motor
```

Karena `web_interface.launch.py` tidak menjalankan node LiDAR sendiri, driver
LiDAR harus sudah berjalan atau `lidar_start_command` harus diisi. Contoh
generik berikut wajib disesuaikan dengan model, baudrate, dan port robot:

```bash
ros2 launch diablo_web_interface web_interface.launch.py \
  lidar_start_command:="ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p serial_port:=/dev/ttyUSBx -p serial_baudrate:=256000"
```

Jangan gunakan `/dev/ttyUSBx` sebelum memeriksa apakah port itu milik LiDAR
atau U2D2.

### Profil B — Web HMI + odometry roda lokal + Nav2

Gunakan setelah frame, `/scan`, odometry, costmap, dan map sudah dikonfigurasi:

```bash
ros2 launch diablo_web_interface nav2_web.launch.py \
  map:=/absolute/path/to/map.yaml
```

Untuk saat ini `map:=...` sebaiknya menunjuk ke map nyata. File fallback
`diablo_web_interface/maps/empty.yaml` hanya untuk memeriksa apakah launch bisa
dimulai.

### Profil C — Simulasi full-body

Profil ini tidak membuka serial dan tidak menggerakkan hardware:

```bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Detailnya ada di [docs/packages/diablo_full_body_moveit_config.md](docs/packages/diablo_full_body_moveit_config.md).

## Urutan startup hardware

Urutan yang disarankan:

1. Pastikan robot berada di area aman dan emergency stop dapat dijangkau.
2. Source ROS dan workspace.
3. Jalankan Web HMI.
4. Pastikan model/port LiDAR benar.
5. Tekan **START HARDWARE**.
6. Tunggu status `DIABLO ROS2` menjadi `READY` sebelum memakai Drive Control.
7. Jalankan AMCL/Nav2/Mapping hanya setelah topic dan TF diperiksa.

Jangan menjalankan dua instance `diablo_ctrl_node`, dua launch Dynamixel, atau
dua driver LiDAR pada port yang sama. Panduan startup dan pengecekan ada di
[docs/STARTUP.md](docs/STARTUP.md).

## Menjalankan web saat mini PC menyala

Untuk test cepat setelah login, perintah berikut dapat dijalankan di shell
mini PC:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_web_interface web_interface.launch.py
```

`~/.bashrc` dapat menjalankan perintah yang sama, tetapi `.bashrc` hanya
dipanggil oleh shell interaktif dan dapat membuat instance ganda saat terminal
baru dibuka. Untuk deployment boot gunakan service `systemd`; lihat
[docs/STARTUP.md](docs/STARTUP.md). Web otomatis aktif tidak berarti motor
otomatis diberi command: hardware tetap dikunci sampai gate HMI dibuka.

## Akses dari laptop

Default backend listen pada semua interface di port `8000`:

```text
http://IP_MINI_PC:8000
```

Untuk development frontend React di laptop:

```bash
cd ~/diablo_ws/src/diablo_web_interface
npm install
VITE_ROBOT_URL=http://IP_MINI_PC:8000 npm run dev
```

Buka `http://localhost:3000`. Tanpa robot, frontend menampilkan preview lokal.
Detail arsitektur WebSocket/REST ada di
[diablo_web_interface/README.md](diablo_web_interface/README.md) dan
[diablo_web_interface/DEPLOY.md](diablo_web_interface/DEPLOY.md).

## Perintah diagnosis

```bash
ros2 node list
ros2 topic list
ros2 topic echo /diablo/sensor/Motors --once
ros2 topic echo /scan --once
ros2 topic echo /diablo/odometry --once
ros2 service list | grep -E 'start_motor|reset|controller'
ls -l /dev/serial/by-id/
```

Jika tidak ada message `/diablo/sensor/Motors`, Drive Control tetap terkunci.
Jika `/scan` tidak ada, LiDAR/Navigation Map tidak dapat digunakan.
