# MoveIt full-body Diablo — simulasi dan hardware

Package ini menjalankan model full-body Diablo secara mandiri di `diablo_ws`.
Demo `demo.launch.py` memakai `mock_components/GenericSystem`, sedangkan
`full_body_hardware.launch.py` memakai plugin hardware nyata.

## 1. Build

Jalankan di terminal baru agar tidak mewarisi overlay workspace lain:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  diablo_body \
  diablo_ctrl \
  diablo_base_hardware \
  diablo_localization \
  diablo_goal_controller \
  diablo_full_body_description \
  diablo_moveit_bridge \
  diablo_full_body_moveit_config
source install/setup.bash
```

Setelah build, setiap terminal baru untuk menjalankan demo harus melakukan:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
```

Jangan source `diablopc_ws` untuk demo ini.

## 2. Uji Step 3 di robot

Perubahan dari workspace development harus disalin dahulu ke
`/home/diablo/diablo_ws`. Jalankan build di robot:

```bash
cd /home/diablo/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  diablo_body \
  diablo_ctrl \
  diablo_base_hardware \
  diablo_localization \
  diablo_goal_controller \
  diablo_full_body_description \
  diablo_full_body_moveit_config
source install/setup.bash
```

Pastikan driver resmi `diablo_ctrl_node` sudah berjalan satu kali dan jangan
membuka port U2D2 yang sama dari proses lain.

Jika driver belum berjalan, jalankan di terminal robot terpisah:

```bash
source /opt/ros/humble/setup.bash
source /home/diablo/diablo_ws/install/setup.bash
ros2 run diablo_ctrl diablo_ctrl_node
```

Pertama, uji mode crawling dan command vehicle langsung tanpa ros2_control.
Pastikan robot berada di lantai, area bebas, dan emergency stop siap:

```bash
ros2 topic pub --once /diablo/MotionCmd motion_msgs/msg/MotionCtrl \
  "{mode_mark: true, value: {up: 1.0}, mode: {pitch_ctrl_mode: false, roll_ctrl_mode: false, height_ctrl_mode: false, stand_mode: false, jump_mode: false, split_mode: false}}"
ros2 topic pub --once /diablo/MotionCmd motion_msgs/msg/MotionCtrl \
  "{mode_mark: false, value: {forward: 0.0, left: 0.0, up: 1.0, roll: 0.0, pitch: 0.0, leg_split: 0.0}}"
```

Setelah uji terisolasi berhasil, launch hardware lengkap dengan kedua U2D2:

```bash
ros2 launch diablo_full_body_moveit_config full_body_hardware.launch.py \
  use_mock_hardware:=false \
  arm_port_name:=/dev/u2d2_arm \
  hand_port_name:=/dev/u2d2_hand \
  use_ekf:=false \
  use_local_odom:=true \
  start_move_group:=false
```

Dengan konfigurasi default `use_ekf:=false use_local_odom:=true`, launch
menjalankan node `local_odom`. Node ini membaca odom mentah
`/diablo_base_controller/odom`, menerbitkan `/diablo/odometry`, dan memiliki
TF `odom -> diablo_base_link` yang dapat di-reset. Odom mentah tetap tersedia
untuk diagnosis. EKF tidak diperlukan untuk uji navigasi ringan ini; hanya
aktif jika `use_ekf:=true` diberikan secara eksplisit.

Pastikan driver resmi sudah berjalan dan frame IMU default
`diablo_robot` memang sebidang dengan `diablo_base_link`. Jika posisi atau
orientasi sensor berbeda, berikan transform saat launch, misalnya:

```bash
imu_x:=0.0 imu_y:=0.0 imu_z:=0.0 \
imu_roll:=0.0 imu_pitch:=0.0 imu_yaw:=0.0
```

Jika udev rule sudah membuat nama tetap, ganti dua argumen port dengan nama
tersebut. Jangan menjalankan dua node yang membuka U2D2 yang sama.

Untuk menguji adapter base saja (tidak membuka U2D2-A/B untuk lengan),
gunakan terminal lain:

```bash
ros2 launch diablo_full_body_moveit_config full_body_hardware.launch.py \
  use_mock_hardware:=false \
  enable_arm_hardware:=false \
  enable_base_hardware:=true \
  start_arm_controllers:=false \
  start_base_controller:=true \
  use_ekf:=false \
  use_local_odom:=true \
  start_move_group:=false
```

Di terminal ketiga, pantau command yang diterjemahkan:

```bash
ros2 topic echo /diablo/MotionCmd
```

Reset pose hanya jika ingin menjadikan posisi robot saat ini sebagai origin
baru. Perintah ini tidak dijalankan otomatis saat startup, sehingga jika tidak
dikirim pose lokal tetap berlanjut ketika robot dipindahkan untuk tes berikutnya:

```bash
ros2 topic pub --once -w 1 /diablo/reset_pose std_msgs/msg/Bool "{data: true}"
```

Atau melalui service yang juga dipakai web HMI:

```bash
ros2 service call /diablo/reset_odom std_srvs/srv/Trigger "{}"
```

Pantau hasilnya:

```bash
ros2 topic echo /diablo/odometry --once
ros2 run tf2_ros tf2_echo odom diablo_base_link
```

Jika ingin melewati wrapper dan melihat odom mentah secara langsung, gunakan
`use_local_odom:=false`; dalam mode itu controller kembali memiliki TF
`odom -> diablo_base_link` dan topic mentahnya adalah
`/diablo_base_controller/odom`.

Di terminal keempat, beri kecepatan sangat rendah selama sekitar dua detik dan
hentikan publisher dengan `Ctrl-C`:

```bash
ros2 topic pub -r 5 /diablo_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Setelah itu, laporkan apakah robot tetap crawling, bergerak lurus, dan apakah
`/diablo_base_controller/odom` serta feedback roda masuk akal:

```bash
ros2 topic echo /diablo_base_controller/odom --once
ros2 topic echo /diablo/sensor/Motors --once
```

Dengan tanda terkalibrasi `left_feedback_sign:=1.0` dan
`right_feedback_sign:=1.0`, kedua velocity encoder mentah positif ketika robot
bergerak maju. Perintah maju positif harus membuat `x` pada
`/diablo/odometry` bertambah dan yaw tetap dekat nol. Jika kabel, firmware,
atau konfigurasi controller berubah, ulangi uji ini sebelum mengganti tanda.

## 4. Jalankan MoveIt dan RViz

Di Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Tunggu sampai log berikut muncul:

```text
Configured and activated `joint_state_broadcaster`, `diablo_base_controller`,
`left_arm_controller`, and `right_arm_controller`

You can start planning now!
```

Untuk menjalankan tanpa RViz:

```bash
ros2 launch diablo_full_body_moveit_config demo.launch.py use_rviz:=false
```

## 5. Group yang tersedia

| Group | Fungsi |
|---|---|
| `left_manipulator` | IK posisi lengan kiri |
| `right_manipulator` | IK posisi lengan kanan |
| `left_arm` / `right_arm` | Joint-space planning lengan |
| `full_body` | Joint-space planning seluruh robot |

`left_manipulator` dan `right_manipulator` saat ini berisi tiga joint arm:
shoulder pitch, shoulder roll, dan elbow. Solver IK memakai KDL dengan
`position_only_ik: true`, sehingga target XYZ dipakai tetapi orientasi belum
ditegakkan. Group `full_body` tidak memiliki satu rantai serial, jadi group
ini tidak digunakan untuk pose IK.

## 6. Plan dan execute dari RViz

Pada panel `MotionPlanning` di RViz:

1. Pilih `Planning Group: left_manipulator`.
2. Pilih pose goal atau gunakan marker goal jika tersedia.
3. Tekan `Plan` untuk membuat trajectory.
4. Tekan `Execute` atau `Plan & Execute` untuk mengirim trajectory ke
   `left_arm_controller` atau `right_arm_controller` sesuai sisi target.

Mode `full_body` digunakan untuk joint-space planning, bukan untuk marker IK
tangan. Mesh robot tidak digerakkan dengan cara drag langsung; yang digeser
adalah interactive marker goal milik MotionPlanning.

## 7. Kirim koordinat melalui IK

Cara ini adalah cara yang paling jelas untuk mengirim target koordinat.
Koordinat menggunakan satuan meter dan frame `torso_link`.

### 7.1 Jalankan bridge IK

Di Terminal 2, setelah Terminal 1 aktif:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py \
  execute:=true
```

`execute:=true` berarti hasil plan dikirim ke controller simulasi. Jika hanya
ingin membuat plan tanpa menggerakkan controller, gunakan:

```bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py \
  execute:=false
```

### 7.2 Kirim target lengan kiri

Di Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 topic pub --once /diablo/ik_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: torso_link}, pose: {position: {x: 0.033, y: 0.338, z: 0.152}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Nilai `x`, `y`, dan `z` dapat diganti. Orientation quaternion wajib valid,
tetapi dengan konfigurasi sekarang orientasi tidak digunakan oleh solver IK.

Log yang menandakan berhasil:

```text
IK succeeded for joints: ...
MoveIt request executed successfully
```

### 7.3 Kirim target lengan kanan

Hentikan bridge kiri dengan `Ctrl+C`, lalu jalankan di Terminal 2:

```bash
ros2 launch diablo_full_body_moveit_config ik_pose_bridge.launch.py \
  group_name:=right_manipulator \
  ik_link_name:=right_arm_tip_link \
  execute:=true
```

Contoh target kanan menggunakan `y` negatif:

```bash
ros2 topic pub --once /diablo/ik_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: torso_link}, pose: {position: {x: 0.033, y: -0.338, z: 0.152}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 8. Cek hasil IK dan controller

Daftar topic dan service yang berguna:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /diablo_full_body_ik_bridge/ik_solution --once
ros2 service list | grep compute_ik
```

Jika tidak ada gerakan, periksa hal berikut:

- `demo.launch.py` harus tetap berjalan.
- Log harus menyatakan controller sudah `Configured and activated`.
- `execute:=true` harus dipakai jika ingin simulasi bergerak.
- Target harus dalam meter dan berada relatif terhadap `torso_link`.
- Gunakan `left_manipulator` atau `right_manipulator`, bukan `full_body`, untuk
  target pose IK.

## 9. MoveIt Setup Assistant

File URDF/Xacro standalone yang dipakai adalah:

```text
~/diablo_ws/src/diablo_full_body_description/description/urdf/diablo_full_body.urdf.xacro
```

Untuk membukanya:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant \
  --urdf_path "$(ros2 pkg prefix diablo_full_body_description)/share/diablo_full_body_description/description/urdf/diablo_full_body.urdf.xacro"
```

## 10. Catatan warna merah di RViz

URDF standalone tidak mendefinisikan material merah. Jika link terlihat merah,
MoveIt biasanya sedang menampilkan link yang dianggap collision. Di display
`MotionPlanning`, coba nonaktifkan `Show Robot Collision` pada bagian
`Scene Robot` untuk membedakan warna visual dari warna collision.
