# diablo_full_body_description

Standalone full-body description untuk Diablo. Package ini menyalin mesh dari
workspace PC, tidak memakai `$(find ...)`, dan menyimpan pilihan hardware di
dalam URDF yang sama.

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

Model menghasilkan 38 link dan 37 joint setelah ekspansi xacro. Secara default
URDF memakai `mock_components/GenericSystem` untuk simulasi. Launch hardware
meneruskan `use_mock_hardware:=false`, lalu memakai `DiabloSystemHardware`
untuk roda dan dua instance `dynamixel_hardware_interface` untuk U2D2-A/B.

Mapping hardware upper-body:

```text
U2D2-A (/dev/ttyUSB1): IDs 1,2,3,6,7,8 untuk lengan.
U2D2-B (/dev/ttyUSB2): IDs 4,5,9,10 untuk wrist/thumb Seed Robotics.
IDs 11,12 untuk leher tidak dimiliki ros2_control karena dipakai human detection.
Baudrate kedua bus: 1000000.
```

Port dapat dioverride dari launch karena nama udev rule dapat berubah.
