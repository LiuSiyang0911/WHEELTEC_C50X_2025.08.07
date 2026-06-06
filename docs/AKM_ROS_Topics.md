# AKM Extended ROS Topics

本文档汇总 `wheeltec14` 新解析器从 STM32 AKM 72B 扩展遥测帧发布的三个新话题。

旧链路仍然保留：

```text
/odom
/imu
/PowerVoltage
```

旧话题来自扩展帧中内嵌的原 24B legacy 数据，主要用于兼容现有导航、显示和调试节点。新话题用于上位机重新建模、标定、融合和诊断。

## 总览

| Topic | Message | 频率 | 主要用途 |
|---|---|---:|---|
| `/wheeltec/akm_state` | `turn_on_wheeltec_robot/AkmState` | 约 100 Hz | 时间戳、编码器、轮速、转向反馈，供 wheel / Ackermann 因子使用 |
| `/wheeltec/control_debug` | `turn_on_wheeltec_robot/ControlDebug` | 约 100 Hz | 控制目标、legacy 车体速度、PWM 输出，供控制调参和实验解释使用 |
| `/wheeltec/chassis_diagnostics` | `turn_on_wheeltec_robot/ChassisDiagnostics` | 约 100 Hz | 电池、超时、低压、自检、包错误计数，供数据质量判断使用 |

当前实测结果：

```text
/wheeltec/akm_state             99.98 Hz
/wheeltec/control_debug         约 100 Hz
/wheeltec/chassis_diagnostics   约 100 Hz
seq_id 重算丢帧                  0
checksum_error_count             0
legacy_error_count               0
```

## `/wheeltec/akm_state`

主数据源。用于轮速/编码器/转向角约束，后续 LiDAR-IMU-Wheel-Ackermann 融合应优先订阅该话题。

Message:

```text
turn_on_wheeltec_robot/AkmState
```

字段：

| 字段 | 类型 | 单位 | 来源/说明 |
|---|---|---|---|
| `header.stamp` | `time` | ROS 时间 | 上位机解析并发布消息时的 ROS 时间 |
| `header.frame_id` | `string` | - | 默认 `base_footprint` |
| `seq_id` | `uint16` | - | STM32 扩展帧序号，用于检测丢包，16 bit 回绕 |
| `control_tick_us` | `uint32` | us | STM32 TIM6 1 MHz 计数，32 bit 回绕 |
| `dt_us` | `uint16` | us | STM32 相邻扩展帧间隔，首帧为 0 |
| `left_encoder_delta` | `int32` | tick/control cycle | 左后轮原始编码器增量 |
| `right_encoder_delta` | `int32` | tick/control cycle | 右后轮原始编码器增量，已按 AKM 方向约定取符号 |
| `left_wheel_speed` | `float32` | m/s | STM32 内部左轮反馈速度 |
| `right_wheel_speed` | `float32` | m/s | STM32 内部右轮反馈速度 |
| `steering_feedback_raw` | `int16` | raw | 舵机滑轨/反馈原始值 |
| `steering_target_raw` | `int16` | raw | STM32 舵机目标原始值 |
| `steering_angle` | `float32` | rad | STM32 标定转换后的前轮转角；有效性看 status bit 5 |
| `steering_pwm` | `int16` | PWM count | 最近一次舵机 PWM 输出 |
| `status_flags` | `uint16` | bitmask | 见本文“状态位” |
| `control_mode` | `uint8` | enum/raw | STM32 `robot_control.ControlMode` 原值 |
| `robot_type` | `uint8` | enum/raw | STM32 运行时识别的 AKM 子型号 |

典型用途：

```text
wheel encoder factor:
  s_{k+1} - s_k ~= encoder_delta * wheel_scale

wheel speed factor:
  v ~= (left_wheel_speed + right_wheel_speed) / 2

Ackermann factor:
  yaw_rate ~= v / wheel_base * tan(steering_angle)

nonholonomic factor:
  v_y ~= 0
```

订阅建议：

- 使用 `control_tick_us` 和 `seq_id` 作为底盘时序主索引。
- 使用 `header.stamp` 做 ROS 消息接收/发布时间，不要把它等同于 STM32 控制周期时间。
- 离线标定时优先保存 `steering_feedback_raw` 和 `steering_angle`，不要只保存角度。
- `status_flags & 0x0020 == 0` 时，不应把 `steering_angle` 当作有效转角约束。

## `/wheeltec/control_debug`

控制诊断数据。它不建议直接进入位姿估计主因子，但非常适合解释速度波动、转向偏差、控制饱和和实验异常。

Message:

```text
turn_on_wheeltec_robot/ControlDebug
```

字段：

| 字段 | 类型 | 单位 | 来源/说明 |
|---|---|---|---|
| `header.stamp` | `time` | ROS 时间 | 上位机解析并发布消息时的 ROS 时间 |
| `header.frame_id` | `string` | - | 默认 `base_footprint` |
| `seq_id` | `uint16` | - | STM32 扩展帧序号 |
| `control_tick_us` | `uint32` | us | STM32 控制周期计数 |
| `target_vx` | `float32` | m/s | STM32 当前目标 x 线速度 |
| `target_vy` | `float32` | m/s | STM32 当前目标 y 线速度；AKM 通常为 0 |
| `target_vz` | `float32` | rad/s | STM32 当前目标 z 角速度 |
| `legacy_vx` | `float32` | m/s | 原 24B legacy 车体 x 速度 |
| `legacy_vy` | `float32` | m/s | 原 24B legacy 车体 y 速度 |
| `legacy_vz` | `float32` | rad/s | 原 24B legacy 车体 z 角速度 |
| `motor_left_pwm` | `int16` | PWM count | 左电机最近输出 PWM |
| `motor_right_pwm` | `int16` | PWM count | 右电机最近输出 PWM |
| `steering_pwm` | `int16` | PWM count | 舵机最近输出 PWM |
| `status_flags` | `uint16` | bitmask | 见本文“状态位” |
| `control_mode` | `uint8` | enum/raw | STM32 `robot_control.ControlMode` 原值 |

典型用途：

- 判断速度波动来自编码器反馈、目标变化还是 PWM 输出变化。
- 判断低速直线时左右电机 PWM 是否长期不对称。
- 判断控制器是否饱和或舵机是否长时间有目标/反馈偏差。
- 对异常轨迹段降低融合权重或剔除。

## `/wheeltec/chassis_diagnostics`

底盘健康状态和通信质量诊断。建议所有上层融合、记录、实验控制节点都订阅或记录该话题。

Message:

```text
turn_on_wheeltec_robot/ChassisDiagnostics
```

字段：

| 字段 | 类型 | 单位 | 来源/说明 |
|---|---|---|---|
| `header.stamp` | `time` | ROS 时间 | 上位机解析并发布消息时的 ROS 时间 |
| `header.frame_id` | `string` | - | 默认 `base_footprint` |
| `seq_id` | `uint16` | - | STM32 扩展帧序号 |
| `control_tick_us` | `uint32` | us | STM32 控制周期计数 |
| `battery_voltage` | `float32` | V | 原 24B legacy 电池电压 |
| `flag_stop` | `bool` | - | STM32 软件失能/停车标志 |
| `command_timeout` | `bool` | - | STM32 认为上位机控制命令超时 |
| `low_voltage` | `bool` | - | STM32 低压状态 |
| `self_check_error` | `bool` | - | STM32 启动自检错误 |
| `steering_angle_valid` | `bool` | - | `steering_angle` 是否有效 |
| `status_flags` | `uint16` | bitmask | 原始状态位 |
| `packet_drop_count` | `uint32` | frames | ROS 解析器按 `seq_id` 检测到的累计丢帧数 |
| `checksum_error_count` | `uint32` | frames | 扩展帧 BCC 校验失败累计数 |
| `legacy_error_count` | `uint32` | frames | 扩展帧内嵌 24B legacy 帧校验失败累计数 |

可信度建议：

- `checksum_error_count` 增长：说明串口帧损坏或解析失步，应降低该时间段数据可信度。
- `legacy_error_count` 增长：说明扩展帧虽然通过，但内嵌 legacy 数据异常，应检查 STM32 打包逻辑。
- `packet_drop_count` 增长：说明 `seq_id` 不连续，应记录时间段并检查串口/CPU 占用。
- `command_timeout == true`：说明底盘可能已停止响应上层速度命令。
- `low_voltage == true` 或 `self_check_error == true`：上层估计和控制应进入降级或停止策略。

## 状态位

`status_flags` 当前定义：

| Bit | Mask | 名称 | 含义 |
|---:|---:|---|---|
| 0 | `0x0001` | `FLAG_STOP` | STM32 `robot_control.FlagStop` 为真 |
| 1 | `0x0002` | `COMMAND_TIMEOUT` | STM32 控制命令超时 |
| 2 | `0x0004` | `LOW_VOLTAGE` | 电池电压低，且未处于允许回充状态 |
| 4 | `0x0010` | `SELF_CHECK_ERROR` | STM32 启动自检错误 |
| 5 | `0x0020` | `STEERING_ANGLE_VALID` | `steering_angle` 已由 AKM 转向反馈转换，建议可用于约束 |
| 6 | `0x0040` | `LEGACY_FRAME_VALID` | 扩展帧内嵌 24B legacy 数据有效 |

实测静止状态常见值：

```text
status_flags = 0x61 = FLAG_STOP | STEERING_ANGLE_VALID | LEGACY_FRAME_VALID
```

## 与旧话题的关系

旧话题继续用于兼容：

| Topic | 数据来源 | 说明 |
|---|---|---|
| `/odom` | 扩展帧内嵌 24B legacy `vx/vy/vz` | STM32 已融合/计算后的车体速度积分；保留旧行为 |
| `/imu` | 扩展帧内嵌 24B legacy IMU 原始量转换 | 保留旧 IMU 发布链路 |
| `/PowerVoltage` | 扩展帧内嵌 24B legacy 电压 | 低频发布，实测约 8.33 Hz |

不要把 `/odom` 当成后续高精度融合的唯一轮速来源。建议新估计节点直接使用：

```text
/wheeltec/akm_state
/wheeltec/control_debug
/wheeltec/chassis_diagnostics
```

## 验证命令

远端 wheeltec14 上：

```bash
source /opt/ros/noetic/setup.bash
source /home/wheeltec/wheeltec_robot/devel/setup.bash

rostopic hz /wheeltec/akm_state
rostopic hz /wheeltec/control_debug
rostopic hz /wheeltec/chassis_diagnostics

rostopic echo -n 1 /wheeltec/akm_state
rostopic echo -n 1 /wheeltec/control_debug
rostopic echo -n 1 /wheeltec/chassis_diagnostics
```

期望：

```text
/wheeltec/akm_state             ~100 Hz
/wheeltec/control_debug         ~100 Hz
/wheeltec/chassis_diagnostics   ~100 Hz
checksum_error_count            不增长
legacy_error_count              不增长
packet_drop_count               不增长
```

## 后续适配建议

推荐下游节点优先适配顺序：

1. 数据记录节点：记录三个新话题和旧 `/odom`、`/imu`、`/PowerVoltage`，用于离线对比。
2. 标定节点：使用 `encoder_delta`、`wheel_speed`、`steering_feedback_raw`、`steering_angle` 做轮径、左右轮 scale、转向零位标定。
3. 融合节点：将 `/wheeltec/akm_state` 用作 wheel factor、Ackermann factor、nonholonomic factor 输入。
4. 诊断/权重节点：根据 `/wheeltec/chassis_diagnostics` 和 `/wheeltec/control_debug` 动态降低异常时间段权重。
