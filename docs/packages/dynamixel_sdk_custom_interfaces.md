# `dynamixel_sdk_custom_interfaces`

Interface contoh dari ROBOTIS SDK:

```text
msg/SetPosition
  uint8 id
  int32 position

srv/GetPosition
  request: uint8 id
  response: int32 position
```

Package ini digunakan oleh `dynamixel_sdk_examples`, bukan oleh command
teleop Diablo utama.

Build dan cek:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dynamixel_sdk_custom_interfaces
source install/setup.bash
ros2 interface show dynamixel_sdk_custom_interfaces/msg/SetPosition
ros2 interface show dynamixel_sdk_custom_interfaces/srv/GetPosition
```
