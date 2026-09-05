# Diablo localization

This package starts a planar `robot_localization` EKF for the real Diablo
base. It fuses wheel and IMU rates into `/odometry/filtered` and owns the
`odom -> diablo_base_link` transform when enabled by
`full_body_hardware.launch.py`.

The filter deliberately does not fuse absolute wheel pose or absolute IMU
quaternion. Its local origin is therefore stable and resettable:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

The equivalent service is used by the web HMI:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

No reset occurs at startup. Without either command, the running EKF keeps its
current pose, which is useful when moving the robot to another test location.
The reset command defines the robot's current position as `(x, y)=(0, 0)` and
its current heading as `theta=0`; subsequent `x` is forward, `y` is left, and
positive `theta` is counter-clockwise.

The default static IMU transform is identity between `diablo_base_link` and
`diablo_robot`. Override `imu_x`, `imu_y`, `imu_z`, `imu_roll`, `imu_pitch`,
and `imu_yaw` in the hardware launch if the physical IMU mounting differs.
