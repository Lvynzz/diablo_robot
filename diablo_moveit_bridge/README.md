# diablo_moveit_bridge

This package connects a `PoseStamped` target to MoveIt. It first calls
MoveIt's `/compute_ik` service, then sends the resulting joint target to the
MoveGroup action for collision-aware planning. The current configuration uses
three joints per arm and a fixed arm tip at the wrist/hand base; wrist and
hand joints are intentionally excluded. It does not access the serial port
directly.

The default is deliberately **plan only**. Hardware execution requires a
real `ros2_control` `FollowJointTrajectory` action server whose joint names,
signs, offsets, limits, and current-state feedback match the MoveIt model.

If the requested position has no IK solution, the default fallback mode tests
nearby positions in increasing distance order. The first successful candidate
is used, and the selected pose is published on
`/ik_moveit_bridge/accepted_target`. This is a sampled approximation of the
closest reachable point. The search now uses radial shells and then bisects
between the original failed target and the first valid candidate. It is
controlled by `fallback_search_radius_m`, `fallback_search_step_m`,
`fallback_max_candidates`, and `fallback_refinement_steps`.

## Build

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select diablo_moveit_bridge
source install/setup.bash
```

## Run with the standalone full-body simulation

The recommended launch is the one in `diablo_full_body_moveit_config`. Start
the standalone MoveIt demo first:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Then, in another terminal, start this bridge:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py
```

The bridge defaults to plan-only. Use `execute:=true` only for the simulated
mock controller:

```bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py execute:=true
```

The same node can also be launched directly from this package when another
MoveIt session already provides `/compute_ik` and `/move_action`:

```bash
ros2 run diablo_moveit_bridge ik_moveit_bridge
```

Disable the nearest-target fallback if strict target rejection is preferred:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py fallback_enabled:=false
```

For diagnosis only, collision rejection can be disabled temporarily:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py \
  avoid_collisions:=false
```

If this makes IK succeed, inspect the Allowed Collision Matrix and collision
meshes. Do not use this mode for real hardware.

Publish an arm-tip target in the MoveIt planning frame (`torso_link` in the
current three-DOF upper-only model):

```bash
ros2 topic pub --once /diablo/ik_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: torso_link}, pose: {position: {x: 0.033, y: 0.338, z: 0.152}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

The IK solution is published on `~/ik_solution` relative to the bridge node.

Use the right arm by changing both group and tip:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py \
  group_name:=right_manipulator ik_link_name:=right_arm_tip_link
```

Do not enable execution until the real upper-arm controller is active and
the mapping has been verified. Only then can the launch be started with:

```bash
ros2 launch diablo_moveit_bridge ik_moveit_bridge.launch.py execute:=true
```
