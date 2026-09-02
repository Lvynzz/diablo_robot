# `dynamixel_sdk_examples`

Contoh node direct SDK untuk satu motor Dynamixel. Ini adalah alat diagnosis,
bukan controller Diablo dan bukan bagian dari Web HMI.

## Menjalankan contoh

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 run dynamixel_sdk_examples read_write_node
```

Source contoh memakai default:

```text
device  : /dev/ttyUSB0
baudrate: 57600
ID      : 1
protocol: 2.0
```

Periksa source sebelum menyalakan torque karena alamat control table berbeda
antar model Dynamixel.

Terminal lain:

```bash
ros2 topic pub --once /set_position \
  dynamixel_sdk_custom_interfaces/msg/SetPosition \
  "{id: 1, position: 1000}"

ros2 service call /get_position \
  dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 1}"
```

Jangan menjalankan contoh ini bersamaan dengan `diablo_bringup` pada U2D2 yang
sama karena keduanya membuka serial port dan dapat berebut torque/control.
