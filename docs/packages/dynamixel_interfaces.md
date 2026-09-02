# `dynamixel_interfaces`

Interface tambahan yang dipakai `dynamixel_hardware_interface` untuk akses data
Dynamixel.

Interface yang tersedia:

- `msg/DynamixelState.msg`
- `srv/GetDataFromDxl.srv`
- `srv/SetDataToDxl.srv`
- `srv/RebootDxl.srv`

Package ini menghasilkan type ROS 2 dan tidak memiliki node mandiri. Build:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dynamixel_interfaces
source install/setup.bash
```

Lihat definisi interface sebelum memanggil service:

```bash
ros2 interface show dynamixel_interfaces/msg/DynamixelState
ros2 interface show dynamixel_interfaces/srv/GetDataFromDxl
ros2 interface show dynamixel_interfaces/srv/SetDataToDxl
ros2 interface show dynamixel_interfaces/srv/RebootDxl
```

Pada robot nyata, reboot/write/register service harus digunakan hanya setelah
ID, alamat control table, dan model Dynamixel dipastikan benar.
