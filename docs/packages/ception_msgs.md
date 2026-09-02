# `ception_msgs`

Package interface telemetry persepsi Diablo. Saat ini menyediakan:

```text
ception_msgs/msg/IMUEuler
  std_msgs/Header header
  float64 roll
  float64 pitch
  float64 yaw
```

Message dipublikasikan oleh `diablo_body` melalui `diablo_ctrl` pada topic
`/diablo/sensor/ImuEuler`.

Build dan cek:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ception_msgs
source install/setup.bash
ros2 interface show ception_msgs/msg/IMUEuler
ros2 topic echo /diablo/sensor/ImuEuler --once
```

Package ini tidak memiliki node mandiri.
