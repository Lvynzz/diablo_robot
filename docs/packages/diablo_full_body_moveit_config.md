# `diablo_full_body_moveit_config`

Konfigurasi MoveIt 2 untuk simulasi full-body Diablo. Demo menggunakan
`mock_components/GenericSystem`, sehingga tidak membuka U2D2 dan tidak
menggerakkan robot fisik.

## Build dan demo

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  diablo_full_body_moveit_config
source install/setup.bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Tanpa RViz:

```bash
ros2 launch diablo_full_body_moveit_config demo.launch.py use_rviz:=false
```

Planning group yang tersedia antara lain `left_manipulator`,
`right_manipulator`, `left_arm`, `right_arm`, dan `full_body`. Gunakan
`left_manipulator`/`right_manipulator` untuk IK pose; `full_body` ditujukan
untuk joint-space planning.

## IK bridge

```bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py \
  execute:=false
```

Kirim target contoh:

```bash
ros2 topic pub --once /diablo/ik_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: torso_link}, pose: {position: {x: 0.033, y: 0.338, z: 0.152}, orientation: {w: 1.0}}}"
```

`execute:=true` hanya untuk mock simulation sampai controller hardware dan
mapping joint benar-benar diverifikasi. Lihat README package untuk detail
planning group dan diagnosis IK.
