# `diablo_teleop`

Keyboard teleop sederhana untuk Diablo. Node ini membaca keyboard dari terminal
dan mempublikasikan `motion_msgs/MotionCtrl`.

## Menjalankan

```bash
source /opt/ros/humble/setup.bash
source ~/diablo_ws/install/setup.bash
ros2 run diablo_teleop teleop_node
```

Tekan tombol backtick `` ` `` untuk keluar. Terminal harus tetap fokus agar
input keyboard terbaca.

## Keybind bawaan

| Tombol | Fungsi |
| --- | --- |
| `W/S` | maju / mundur |
| `A/D` | putar kiri / kanan |
| `Q/E` | roll kiri / kanan |
| `R` | reset roll |
| `H/J/K/L` | kontrol tinggi kaki |
| `U/I/O` | pitch |
| `V/B` | mode kontrol tinggi on/off |
| `N/M` | mode kontrol pitch on/off |
| `Z` | standing mode |
| `X` | crawling mode |
| `C` | jump |
| `F/G` | split/dance on/off |

## Dengan Web HMI

Jangan biarkan node ini dan mux sama-sama mempublikasikan ke topic utama.
Jika ingin menjadikannya sumber manual mux:

```bash
ros2 run diablo_teleop teleop_node --ros-args \
  -r diablo/MotionCmd:=/diablo/MotionCmd/manual
```

Untuk penggunaan sehari-hari gunakan Drive Control HMI karena memiliki
watchdog, hardware gate, STOP, dan keybind yang dapat di-remap.
