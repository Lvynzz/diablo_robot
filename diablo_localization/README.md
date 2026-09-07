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
