# `motion_msgs`

Package interface untuk command dan telemetry utama Diablo.

## Message utama

### `MotionCtrl`

```text
bool mode_mark
MovementCtrlData value
MovementCtrlMode mode
```

Jika `mode_mark=false`, field `value` digunakan:

```text
value.forward    # maju/mundur, m/s sesuai SDK
value.left       # yaw kiri/kanan
value.up         # tinggi
value.roll       # roll
value.pitch      # pitch
value.leg_split  # split kaki
```

Jika `mode_mark=true`, field `mode` digunakan untuk standing/crawling dan mode
kontrol postur.

### `LegMotors`

Berisi encoder/posisi/velocity/iq hip, knee, dan wheel kiri-kanan serta panjang
kaki. Web HMI `wheel_odom` membaca `left_wheel_pos`, `right_wheel_pos`, dan
revolution counter dari message ini.

### Message lain

- `CtrlPlot`: header dan `float64 value`.
- `RobotStatus`: control mode, robot mode, error, warning.
- `MovementCtrlData` dan `MovementCtrlMode`: struktur yang dipakai `MotionCtrl`.

## Cek interface

```bash
ros2 interface show motion_msgs/msg/MotionCtrl
ros2 interface show motion_msgs/msg/LegMotors
```

Contoh command nol untuk diagnosis saja:

```bash
ros2 topic pub --once /diablo/MotionCmd motion_msgs/msg/MotionCtrl \
  "{mode_mark: false, value: {forward: 0.0, left: 0.0, up: 0.0, roll: 0.0, pitch: 0.0, leg_split: 0.0}, mode: {stand_mode: true}}"
```

Hindari mengirim command gerak langsung sebelum driver, mode robot, dan area
keselamatan diverifikasi.
