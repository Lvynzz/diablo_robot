# `diablo_goal_controller`

`simple_goal_controller` subscribes to
`/diablo_base_controller/odom` and publishes
`/diablo_base_controller/cmd_vel_unstamped` for a single planar `(x, y)` goal.
It rotates in place when the heading error is large, then drives forward with
a small heading correction.  It stops when the goal tolerance is reached or
odometry becomes stale.

Example:

```bash
ros2 run diablo_goal_controller simple_goal_controller --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0
```

The goal can be changed while running with:

```bash
ros2 param set /simple_goal_controller goal_x 0.5
ros2 param set /simple_goal_controller goal_y 0.8
```

Run it only after `diablo_base_controller` has published its odometry topic:

```bash
ros2 run diablo_goal_controller simple_goal_controller --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0
```
