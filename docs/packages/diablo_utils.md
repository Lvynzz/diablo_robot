# `diablo_utils`

`diablo_utils` adalah library low-level SDK Diablo: serial port, CRC, packet
protocol, telemetry, movement command, dan virtual RC. Package ini menjadi
dependency `diablo_ctrl` dan tidak memiliki executable ROS yang perlu
dijalankan operator.

Build dan penggunaan normal:

```bash
cd ~/diablo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to diablo_ctrl
source install/setup.bash
ros2 run diablo_ctrl diablo_ctrl_node
```

Serial SDK yang dipakai driver saat ini diinisialisasi di
`diablo_ctrl.cpp` dengan `/dev/diablo_controller`. Port dapat diganti melalui
parameter `controller_port`, bukan dengan mengubah package web, lalu workspace
perlu dibuild ulang jika binary belum terbaru.

Pengguna aplikasi biasanya tidak perlu memanggil API library ini secara
langsung. Jika mengubah protocol atau timing, lakukan pengujian tanpa robot
terhubung terlebih dahulu dan jangan menaikkan frekuensi telemetry tanpa alasan.
