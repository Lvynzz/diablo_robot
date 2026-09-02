# MoveIt full-body Diablo — panduan simulasi

Package ini menjalankan model full-body Diablo secara mandiri di `diablo_ws`.
Package ini tidak membutuhkan `diablopc_ws`, tidak membuka port serial, dan
tidak menggerakkan robot fisik. Controller yang dipakai adalah
`mock_components/GenericSystem`, sehingga aman untuk menguji plan dan execute.

## 1. Build

Jalankan di terminal baru agar tidak mewarisi overlay workspace lain:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
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

## 2. Jalankan MoveIt dan RViz

Di Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 launch diablo_full_body_moveit_config demo.launch.py
```

Tunggu sampai log berikut muncul:

```text
Configured and activated joint_state_broadcaster
Configured and activated diablo_full_body_controller
You can start planning now!
```

Untuk menjalankan tanpa RViz:

```bash
ros2 launch diablo_full_body_moveit_config demo.launch.py use_rviz:=false
```

## 3. Group yang tersedia

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

## 4. Plan dan execute dari RViz

Pada panel `MotionPlanning` di RViz:

1. Pilih `Planning Group: left_manipulator`.
2. Pilih pose goal atau gunakan marker goal jika tersedia.
3. Tekan `Plan` untuk membuat trajectory.
4. Tekan `Execute` atau `Plan & Execute` untuk mengirim trajectory ke
   `diablo_full_body_controller`.

Mode `full_body` digunakan untuk joint-space planning, bukan untuk marker IK
tangan. Mesh robot tidak digerakkan dengan cara drag langsung; yang digeser
adalah interactive marker goal milik MotionPlanning.

## 5. Kirim koordinat melalui IK

Cara ini adalah cara yang paling jelas untuk mengirim target koordinat.
Koordinat menggunakan satuan meter dan frame `torso_link`.

### 5.1 Jalankan bridge IK

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

### 5.2 Kirim target lengan kiri

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

### 5.3 Kirim target lengan kanan

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

## 6. Cek hasil IK dan controller

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

## 7. MoveIt Setup Assistant

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

## 8. Catatan warna merah di RViz

URDF standalone tidak mendefinisikan material merah. Jika link terlihat merah,
MoveIt biasanya sedang menampilkan link yang dianggap collision. Di display
`MotionPlanning`, coba nonaktifkan `Show Robot Collision` pada bagian
`Scene Robot` untuk membedakan warna visual dari warna collision.
