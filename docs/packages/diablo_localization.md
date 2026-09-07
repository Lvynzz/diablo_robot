# `diablo_localization`

`diablo_localization` menjalankan odometri lokal resettable:

```text
/diablo_base_controller/odom -> local_odom -> /diablo/odometry
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
menerbitkan TF `odom -> diablo_base_link` ketika `use_local_odom:=true`; TF itu
diterbitkan oleh `local_odom`. Set `use_local_odom:=false` jika ingin memakai
odom mentah dan TF controller langsung. EKF hanya aktif bila `use_ekf:=true`
diberikan secara eksplisit.

Setelah reset, koordinat lokal pada `/diablo/odometry` memakai `x` positif ke
depan robot, `y` positif ke kiri, dan heading `theta` dalam radian positif
berlawanan arah jarum jam.
