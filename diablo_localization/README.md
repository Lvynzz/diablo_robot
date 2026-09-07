# Diablo localization

The default hardware launch uses `local_odom`, a resettable local odometry
wrapper around `/diablo_base_controller/odom`. It publishes
`/diablo/odometry` and owns the `odom -> diablo_base_link` transform. The raw
controller odometry remains available for calibration and diagnostics.

The local wrapper does not fuse IMU data or alter the raw wheel odometry. Its
local origin is therefore explicit and resettable:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

The equivalent service is used by the web HMI:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

No reset occurs at startup. Without either command, the local node keeps its
current pose, which is useful when moving the robot to another test location.
The reset command stores the current raw pose as `(x, y, theta)=(0, 0, 0)`;
subsequent `x` is forward, `y` is left, and positive `theta` is
counter-clockwise.

The EKF launch remains available for experiments with
`ros2 launch diablo_localization localization.launch.py`, but it is not used
by `full_body_hardware.launch.py` unless `use_ekf:=true` is explicitly set.

The static IMU transform parameters are only used by the optional EKF launch.
The default local wheel odometry path does not subscribe to IMU data.

## Wheel odometry calibration

Wheel odometry can return to `(0, 0)` in its estimate while the robot is not
physically at the marked start point.  Calibrate the geometry before judging
goal-controller accuracy, especially after tests that include rotation.

First check that there is only one local odometry publisher and one goal
controller:

```bash
ros2 topic info -v /diablo/odometry
ros2 node list | grep -E 'diablo_local_odom|diablo_wheel_odom|simple_goal_controller'
```

For a straight-line test, mark the robot's physical start, reset the local
pose, drive a measured distance, and record both the physical distance
`d_physical` and `/diablo/odometry` distance `d_odom`.  Update the wheel radius
with:

```text
wheel_radius_new = wheel_radius_old * d_physical / d_odom
```

For a rotation test, record the physical yaw change `yaw_physical` and the
odometry yaw change `yaw_odom`.  Update the track width with:

```text
track_width_new = track_width_old * yaw_odom / yaw_physical
```

Use the calibrated wheel radius consistently as `wheel_radius` in the
`diff_drive_controller` configuration and the hardware launch/xacro.  Use
the calibrated track width consistently as `wheel_separation` in the
controller and `track_width` in the hardware launch/xacro.  The legacy
standalone `wheel_odom` launch arguments should use the same values if that
node is enabled.

After calibration, repeat the two-way test: reset at a marked start, send a
goal such as `(0.5, 0.5)`, then send `(0.0, 0.0)` without resetting.  Compare
the final physical position with the mark; wheel odometry alone cannot
correct wheel slip, so a persistent large residual requires an external
position reference such as IMU fusion or lidar.
