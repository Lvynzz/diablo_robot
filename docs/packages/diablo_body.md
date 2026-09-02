# `diablo_body`

`diablo_body` adalah library telemetry, bukan executable yang dijalankan
sendiri. Library ini dipakai oleh `diablo_ctrl` untuk membungkus data SDK
menjadi topic ROS 2:

- `diablo/sensor/Motors` — posisi, kecepatan, arus, dan panjang kaki.
- `diablo/sensor/Imu` — `sensor_msgs/Imu`.
- `diablo/sensor/ImuEuler` — roll, pitch, yaw.
- `diablo/sensor/Battery` — `sensor_msgs/BatteryState`.
- `diablo/sensor/Body_state` — status mode/error/warning Diablo.

Build bersama dependency-nya:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to diablo_ctrl
source install/setup.bash
```

Jangan menjalankan package ini dengan `ros2 run`; tidak ada node mandiri yang
diekspor. Jalankan `ros2 run diablo_ctrl diablo_ctrl_node` untuk mengaktifkan
library ini.

Detail message ada di [motion_msgs](motion_msgs.md) dan
[ception_msgs](ception_msgs.md).
