# DIABLO Interaction 机器人控制交互节点

​	此节点是机器人控制的基础节点，您需要运行 `diablo_ctrl_node` 获取机器人的控制权限。您可以将您的控制指令以自定义msg : `MotionCtrl` 的格式发送到 `/diablo/MotionCmd` 实现控制效果。



## Serial Port Modification 串口修改

​	本机器人使用 udev 固定名称 `/dev/diablo_controller` 进行串口通信。也可以
	通过 ROS 参数覆盖端口，不需要重新编译。

```bash
ros2 run diablo_ctrl diablo_ctrl_node \
  --ros-args -p controller_port:=/dev/diablo_controller
```



## Control instructions 控制指令

| 控制ID               | 数值范围   | 备注                 |
| :------------------- | ---------- | -------------------- |
| CMD_GO_FORWARD(0x08) | ±2.0 m/s   | 负数为向后运动       |
| CMD_GO_LEFT(0x04)    | ±2.0 rad/s | 负数为向右运动       |
| CMD_ROLL_RIGHT(0x09) | ±0.2 rad   | 负数为向左运动       |
| CMD_STAND_UP(0x02)   | 0.0        | 机器人站立           |
| CMD_STAND_DOWN(0x12) | 0.0        | 机器人下蹲           |
|                      |            |                      |
| CMD_PITCH_MODE(0x13) | (0.0，1.0) | 位置模式0，速度模式1 |
| CMD_PITCH(0x03)      | ±0.3 pi    | 位置模式0            |
| CMD_PITCH(0x03)      | ±1.8 rad/s | 速度模式1            |
|                      |            |                      |
| CMD_HEIGH_MODE(0x01) | (0.0，1.0) | 位置模式0，速度模式1 |
| CMD_BODY_UP(0x11)    | 0.0~1.0    | 位置模式0            |
| CMD_BODY_UP(0x11)    | ±0.25 m/s  | 速度模式1            |

> 速度模式，机器人将以指令中数值的速度持续运动。
>
> 位置模式，机器人将以指令中数值代表的固定位置运动。
