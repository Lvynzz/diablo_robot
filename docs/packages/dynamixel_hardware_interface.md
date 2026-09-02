# `dynamixel_hardware_interface`

Plugin `ros2_control` untuk membaca dan mengirim command ke motor Dynamixel
melalui U2D2/USB2Dynamixel. Source package berada pada submodule dengan
README upstream di `dynamixel_hardware_interface/README.md`.

## Peran di workspace

Package ini tidak biasanya dijalankan dengan `ros2 run`. Plugin dipanggil dari
tag `<ros2_control>` di xacro `diablo_bringup`, misalnya:

```xml
<plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
<param name="port_name">/dev/ttyUSB0</param>
<param name="baud_rate">1000000</param>
```

Launch controller yang memakai plugin:

```bash
ros2 launch diablo_bringup six_joint_move.launch.py
```

## Parameter yang perlu diperiksa

- `port_name`: device U2D2.
- `baud_rate`: harus sama dengan konfigurasi motor.
- `number_of_joints`: jumlah joint yang dibaca.
- matrix transmission: pemetaan joint ke transmission.
- model/ID dan control table di xacro.

Cek hasilnya:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Jangan memakai port yang sama untuk driver LiDAR dan U2D2. Jika controller
gagal aktif, periksa power motor, permission `dialout`, baudrate, ID motor,
dan port serial sebelum mengubah gain.
