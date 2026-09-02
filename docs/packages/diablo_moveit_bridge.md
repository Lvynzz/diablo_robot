# `diablo_moveit_bridge`

Node Python yang menerima `geometry_msgs/PoseStamped`, meminta `/compute_ik`
ke MoveIt, lalu meneruskan target joint melalui `/move_action`. Node ini
ditujukan untuk lengan atas Diablo dan default-nya plan-only.

## Menjalankan bersama simulasi

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Terminal 2:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py \
  execute:=false
```

Pakai `execute:=true` hanya pada mock simulation. Untuk kanan:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py \
  group_name:=right_manipulator ik_link_name:=right_arm_tip_link
```

Parameter penting ada di `config/ik_moveit_bridge.yaml`: planning frame,
fallback target, collision checking, timeout, dan velocity scaling.

## Safety

Jangan memakai `execute:=true` pada hardware sebelum controller, joint mapping,
limit, collision model, dan workspace lengan diuji. `avoid_collisions:=false`
hanya untuk diagnosis dan tidak boleh dipakai sebagai mode operasi robot.
