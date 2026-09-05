# `diablo_localization`

`diablo_localization` menjalankan EKF planar berbasis `robot_localization`:

```text
/diablo_base_controller/odom + /diablo/sensor/Imu
              -> /odometry/filtered
```

Launch full-body sudah mengaktifkannya secara default. Untuk reset eksplisit
ke origin lokal, jalankan hanya ketika robot berhenti:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

Atau gunakan service:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

Tidak ada reset otomatis ketika node dimulai. `diablo_base_controller` tidak
lagi menerbitkan TF `odom -> diablo_base_link` ketika `use_ekf:=true`; TF itu
diterbitkan oleh EKF.

Setelah reset, koordinat lokal memakai `x` positif ke depan robot, `y` positif
ke kiri, dan heading `theta` dalam radian positif berlawanan arah jarum jam.
