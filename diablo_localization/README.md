# Diablo localization

The web HMI starts `ekf_hardware.launch.py` when DIABLO hardware is enabled.
That launch starts the SDK driver, a private raw wheel odometry source on
`/diablo_base_controller/odom`, and one `robot_localization` EKF. The EKF is
the only publisher of `odom -> diablo_base_link` and publishes
`/odometry/filtered`, which is the topic used by Nav2 and the web UI.

The reset helper provides an explicit EKF pose reset:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

The equivalent service is used by the web HMI:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

No reset occurs at startup. Without either command, the local node keeps its
current pose, which is useful when moving the robot to another test location.
The reset command sets `(x, y, theta)=(0, 0, 0)`. The web HMI also exposes
separate `/diablo/reset_position` and `/diablo/reset_orientation` services;
they preserve the other two pose components. Subsequent `x` is forward, `y`
is left, and positive `theta` is counter-clockwise.

For a standalone filter (when a raw `/diablo_base_controller/odom` source is
already running), use `ros2 launch diablo_localization localization.launch.py`.

The Diablo profile fuses wheel x/y pose and forward velocity from
`/diablo_base_controller/odom`, plus relative yaw from the onboard IMU at
`/diablo/sensor/Imu`. It deliberately does not fuse wheel yaw or the IMU gyro,
so the IMU is the sole orientation source. The filtered result is published
on `/odometry/filtered`.

The static IMU transform parameters are only used by the optional EKF launch.
The default local wheel odometry path does not subscribe to IMU data.

## Wheel odometry calibration

Wheel odometry can return to `(0, 0)` in its estimate while the robot is not
physically at the marked start point.  Calibrate the geometry before judging
goal-controller accuracy, especially after tests that include rotation.

First check that there is only one filtered odometry publisher and one goal
controller:

```bash
ros2 topic info -v /odometry/filtered
ros2 node list | grep -E 'diablo_ekf_filter|raw_wheel_odom|simple_goal_controller'
```

For a straight-line test, mark the robot's physical start, reset the EKF pose,
drive a measured distance, and record both the physical distance
`d_physical` and `/odometry/filtered` distance `d_odom`. Update the wheel radius
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
goal such as `(0.5, 0.5)`, then send `(0.0, 0.0)` without resetting. Compare
the final physical position with the mark; wheel/IMU EKF alone cannot correct
wheel slip or absolute position, so a persistent large residual requires AMCL
or another external position reference.
