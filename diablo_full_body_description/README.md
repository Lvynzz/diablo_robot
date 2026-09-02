# diablo_full_body_description

Standalone full-body description untuk Diablo. Package ini menyalin mesh dari
workspace PC, tetapi tidak memanggil `diablo_ros2_control`, tidak memakai
`$(find ...)`, dan tidak terhubung ke serial port.

Package ini biasanya dijalankan bersama
`diablo_full_body_moveit_config`. Untuk build dan menjalankan simulasi, lihat:

```text
~/diablo_ws/src/diablo_full_body_moveit_config/README.md
```

## File URDF/Xacro

File yang dipilih di MoveIt Setup Assistant:

```text
description/urdf/diablo_full_body.urdf.xacro
```

Setelah build dan source `diablo_ws`, buka Setup Assistant dengan:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant \
  --urdf_path "$(ros2 pkg prefix diablo_full_body_description)/share/diablo_full_body_description/description/urdf/diablo_full_body.urdf.xacro"
```

Model menghasilkan 38 link dan 37 joint setelah ekspansi xacro. URDF juga
memiliki block `mock_components/GenericSystem` untuk simulasi ros2_control.
