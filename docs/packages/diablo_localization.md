# `diablo_localization`

`diablo_localization` menjalankan satu filter EKF wheel/IMU:

```text
/diablo/sensor/Motors -> raw wheel odom -> /diablo_base_controller/odom
                                      -> EKF -> /odometry/filtered
```

Launch full-body mengaktifkan `local_odom` secara default. Untuk reset
eksplisit ke origin lokal, jalankan hanya ketika robot berhenti:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

Atau gunakan service:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

Tidak ada reset otomatis ketika node dimulai. `diablo_base_controller` tidak
EKF menerbitkan TF `odom -> diablo_base_link` dan `/odometry/filtered`. Jangan
menjalankan `local_odom` atau odom legacy yang menerbitkan TF pada saat yang
sama.

Setelah reset, koordinat lokal pada `/odometry/filtered` memakai `x` positif ke
depan robot, `y` positif ke kiri, dan heading `theta` dalam radian positif
berlawanan arah jarum jam.

## Kalibrasi wheel odometry

Jika odometri kembali ke `(0, 0)` tetapi robot tidak kembali ke tanda fisik
awal, periksa dulu bahwa hanya ada satu publisher odometri:

```bash
ros2 topic info -v /odometry/filtered
ros2 node list | grep -E 'diablo_ekf_filter|raw_wheel_odom|simple_goal_controller'
```

Kalibrasikan gerak lurus terlebih dahulu. Ukur jarak fisik `d_physical` dan
jarak odometri `d_odom`, lalu gunakan:

```text
wheel_radius_new = wheel_radius_old * d_physical / d_odom
```

Untuk kalibrasi putaran, ukur perubahan heading fisik `yaw_physical` dan
heading odometri `yaw_odom`:

```text
track_width_new = track_width_old * yaw_odom / yaw_physical
```

Gunakan hasil yang sama pada `wheel_radius`/`track_width` hardware dan
`wheel_radius`/`wheel_separation` `diff_drive_controller`. Wheel odometry
tidak dapat mengoreksi slip; jika error fisik tetap besar setelah kalibrasi,
diperlukan referensi eksternal seperti IMU fusion atau lidar.

Nilai geometri default saat ini adalah `wheel_radius=0.187` meter dan
`wheel_separation=0.488` meter (`track_width=0.488` pada hardware).
