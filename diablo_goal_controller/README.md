# `diablo_goal_controller`

`simple_goal_controller` subscribes by default to the fused
`/odometry/filtered` topic and publishes
`/diablo_base_controller/cmd_vel_unstamped` for a single planar `(x, y)` goal.
It rotates in place when the heading error is large, then drives forward with
a small heading correction.  When a final heading is enabled, it rotates in
place after reaching `(x, y)`.  It stops when the goal pose is reached or
odometry becomes stale.  The controller does not reset its odometry origin by
default; use the explicit localization reset command when a new test origin
is desired.

Example:

```bash
ros2 run diablo_goal_controller simple_goal_controller --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0 \
  -p use_goal_yaw:=true -p goal_yaw:=0.0
```

The goal can also be sent as one command containing x/y metres and heading in
radians:

```bash
ros2 topic pub --once -w 1 /diablo/goal_pose geometry_msgs/msg/Pose2D \
  "{x: 1.0, y: 0.5, theta: 1.5708}"
```

The pose is interpreted in the `/odometry/filtered` frame: `x` positive is
forward from the current local origin, `y` positive is to the robot's left,
and `theta` is heading in radians, positive counter-clockwise.  After a pose
reset, the current robot heading is defined as `theta=0`.  Set
`odom_topic:=/diablo_base_controller/odom` when testing without the EKF.

The goal can be changed while running with:

```bash
ros2 param set /simple_goal_controller goal_x 0.5
ros2 param set /simple_goal_controller goal_y 0.8
ros2 param set /simple_goal_controller use_goal_yaw true
ros2 param set /simple_goal_controller goal_yaw 1.5708
```

Reset the fused pose only when you want a new local origin.  If this command is
not sent, the running estimator keeps its current pose:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

The equivalent service, also used by the web HMI, is:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

Run the goal controller only after the EKF has published its odometry topic:

```bash
ros2 run diablo_goal_controller simple_goal_controller --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0 -p use_goal_yaw:=false
```
