# `diablo_ctrl`

`diablo_ctrl` adalah driver ROS 2 utama untuk Diablo. Node
`diablo_ctrl_node` membuka SDK/telemetry Diablo, menerima `motion_msgs/MotionCtrl`,
lalu mengirim command ke robot. Node yang sama juga mempublikasikan telemetry
yang dipakai Web HMI dan wheel odometry.

## Menjalankan

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 run diablo_ctrl diablo_ctrl_node
```

Port SDK default driver adalah `/dev/diablo_controller`. Pada robot, udev rule
memetakan nama itu ke USB controller Diablo yang sesuai. Jika hardware memakai
port lain, berikan parameter saat menjalankan node:

```bash
ros2 run diablo_ctrl diablo_ctrl_node --ros-args \
  -p controller_port:=/dev/diablo_controller
```

## Topic

| Arah | Topic | Type |
| --- | --- | --- |
| subscribe | `/diablo/MotionCmd` | `motion_msgs/msg/MotionCtrl` |
| publish | `/diablo/sensor/Motors` | `motion_msgs/msg/LegMotors` |
| publish | `/diablo/sensor/Imu` | `sensor_msgs/msg/Imu` |
| publish | `/diablo/sensor/ImuEuler` | `ception_msgs/msg/IMUEuler` |
| publish | `/diablo/sensor/Battery` | `sensor_msgs/msg/BatteryState` |
| publish | `/diablo/sensor/Body_state` | `motion_msgs/msg/RobotStatus` |

Cek driver:

```bash
ros2 node list
ros2 topic echo /diablo/sensor/Motors --once
ros2 topic info /diablo/MotionCmd
```

Web HMI memakai `/diablo/MotionCmd/manual` lalu mux meneruskannya ke topic
utama. Hindari menjalankan publisher langsung ke topic utama bersamaan dengan
mux kecuali memang sedang melakukan diagnosis.

## Safety

Satu message terakhir disimpan oleh driver dan heartbeat SDK mengirimkannya.
Untuk berhenti, kirim `MotionCtrl` dengan nilai gerak nol atau gunakan tombol
STOP di HMI. Uji pertama dilakukan dengan roda terangkat dan kecepatan rendah.
