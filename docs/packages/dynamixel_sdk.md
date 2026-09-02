# `dynamixel_sdk`

Library C++/Python ROBOTIS untuk komunikasi packet Dynamixel. Source ROS 2
berada di `DynamixelSDK/ros/dynamixel_sdk` dan dipakai oleh
`dynamixel_hardware_interface`.

Library ini tidak dijalankan sebagai node. Build bersama workspace:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dynamixel_sdk
source install/setup.bash
```

Parameter penting ditentukan oleh aplikasi pemakai, bukan oleh package SDK:

- nama port serial;
- protocol version;
- baudrate;
- ID motor;
- alamat control table sesuai model.

Untuk Diablo, lebih aman menggunakan launch `diablo_bringup` karena konfigurasi
ID, joint, controller, dan ros2_control sudah diletakkan di xacro/YAML.
Contoh direct SDK ada di package `dynamixel_sdk_examples`.
