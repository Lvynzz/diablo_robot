# `diablo_full_body_description`

URDF/xacro standalone full-body Diablo beserta mesh visual/collision. Package
ini aman untuk simulasi: tidak membuka serial dan tidak menggerakkan robot.

## Menjalankan display

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_description display.launch.py
```

Argumen:

```bash
ros2 launch diablo_full_body_description display.launch.py \
  gui:=true use_gui:=false rviz:=true
```

File model utama:

```text
description/urdf/diablo_full_body.urdf.xacro
```

Untuk planning gunakan package `diablo_full_body_moveit_config`. Jangan
menganggap model ini sebagai konfigurasi hardware; serial dan controller
nyata berada di `diablo_bringup`.
