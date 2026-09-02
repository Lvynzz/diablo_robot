# `diablo_bringup`

Package ini berisi URDF/xacro, konfigurasi `ros2_control`, launch file, dan
script untuk menghubungkan joint Dynamixel Diablo ke U2D2. Package ini dipakai
oleh Web HMI untuk menjalankan controller six-joint upper body.

## Prasyarat

- ROS 2 Humble
- `ros2_control` dan `ros2_controllers`
- `dynamixel_sdk`
- `dynamixel_hardware_interface`
- U2D2, power motor, serta akses user ke grup `dialout`

Build dari root workspace:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select diablo_bringup
source install/setup.bash
```

## Launch file

| Launch | Fungsi |
| --- | --- |
| `six_joint_move.launch.py` | Enam joint position controller untuk ID `1, 2, 3, 6, 7, 8` |
| `seven_joint_move.launch.py` | Tujuh joint trajectory controller |
| `joint2_move.launch.py` | Satu joint, mendukung `use_dummy` |
| `joint23_move.launch.py` | Joint ID 2 dan 3 |
| `joint123_move.launch.py` | Joint ID 1, 2, dan 3 |
| `read_joints.launch.py` | Publikasi state enam joint tanpa position controller |
| `two_joint_read.launch.py` | Publikasi state dua joint |
| `three_joint_read.launch.py` | Publikasi state tiga joint |

Contoh six-joint hardware:

```bash
ros2 launch diablo_bringup six_joint_move.launch.py
```

Contoh joint2 dummy/simulasi:

```bash
ros2 launch diablo_bringup joint2_move.launch.py \
  use_dummy:=true port_name:=/dev/ttyUSB0 baud_rate:=1000000
```

`joint2_move.launch.py` dan `joint23_move.launch.py` menerima `port_name` dan
`baud_rate`. Launch six/seven/read memakai nilai yang tertulis di xacro, jadi
ubah xacro jika port hardware berbeda. Versi saat ini menggunakan
`/dev/ttyUSB0` dan baudrate `1000000` pada xacro utama.

## Cek controller

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Sebelum menjalankan hardware, pastikan robot terangkat/aman dan emergency stop
tersedia. Jangan menjalankan dua `ros2_control_node` yang memakai port U2D2
yang sama.

## File penting

- `urdf/*.urdf.xacro`: model joint dan konfigurasi plugin hardware.
- `config/*_controllers.yaml`: controller manager dan daftar joint.
- `ID_dynamixel`: catatan ID motor yang digunakan.
- `scripts/`: teleop/monitor joint untuk eksperimen terpisah dari Web HMI.
