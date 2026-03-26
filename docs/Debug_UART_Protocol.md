# WHEELTEC C50X Debug UART Protocol

## 1. 概述

调试串口模块通过 `UART1 + DMA` 实现实时数据输出和命令接收，主要用于 AKM 车型速度环调试、PI/LADRC 参数在线整定以及开环/闭环对比分析。

通信方向：
- TX（Robot -> PC）：周期发送实时数据帧、控制状态帧，按需发送参数帧
- RX（PC -> Robot）：在线设置 PID、LADRC、速度限制、平滑参数、A/B 两轮目标速度或目标 PWM

当前实现对应源码：
- [debug_uart.h](C:/Users/86136/Desktop/work/work/WHEELTEC_C50X_2025.08.07/HARDWARE/Inc/debug_uart.h)
- [debug_uart.c](C:/Users/86136/Desktop/work/work/WHEELTEC_C50X_2025.08.07/HARDWARE/debug_uart.c)
- [balance_task.h](C:/Users/86136/Desktop/work/work/WHEELTEC_C50X_2025.08.07/BALANCE/Inc/balance_task.h)
- [balance_task.c](C:/Users/86136/Desktop/work/work/WHEELTEC_C50X_2025.08.07/BALANCE/balance_task.c)

## 2. 硬件配置

| 项目 | 配置 |
|------|------|
| 串口 | `USART1` |
| 波特率 | `115200` |
| 数据位 | `8` |
| 停止位 | `1` |
| 校验位 | 无 |
| TX 引脚 | `PA9` |
| RX 引脚 | `PA10` |
| TX DMA | `DMA2_Stream7_Channel4` |
| 发送方式 | DMA 单次发送 |
| 接收方式 | USART1 RXNE 中断 + 状态机解析 |

## 3. 通用帧格式

### 3.1 字节序

所有多字节数据均为小端序，`float` 使用 IEEE-754 单精度 32 位格式。

### 3.2 TX 帧格式

```text
[0]     0xAA
[1]     0x55
[2]     FrameID
[3..N-2] Payload
[N-1]   Checksum
```

说明：
- `Checksum = XOR(bytes[0 .. N-2])`

### 3.3 RX 帧格式

```text
[0]         0xAA
[1]         0x55
[2]         CmdID
[3]         Length
[4..3+L]    Payload
[4+L]       Checksum
```

说明：
- `Checksum = XOR(bytes[0 .. 3+Length])`
- 当前接收缓冲区 `DEBUG_RX_BUF_LEN = 32`
- RX 状态机超时阈值约为 `5` 个 `Balance_task` 周期，当前约 `50ms`

## 4. TX 帧定义

### 4.1 数据帧 `FrameID = 0x01`

- 总长度：`40` 字节
- 发送频率：由 `Balance_task` 在 `100Hz` 周期内触发
- 在 `AKM_CAR` 构建下，和控制状态帧 `0x03` 交替发送

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x01` |
| 3-6 | 4 | float | t_raw_A | 电机 A 的 T 法原始速度，单位 `m/s` |
| 7-10 | 4 | float | t_raw_B | 电机 B 的 T 法原始速度，单位 `m/s` |
| 11-14 | 4 | float | m_raw_A | 电机 A 的 M 法原始速度，单位 `m/s` |
| 15-18 | 4 | float | m_raw_B | 电机 B 的 M 法原始速度，单位 `m/s` |
| 19-22 | 4 | float | final_A | A 轮当前闭环反馈速度，单位 `m/s` |
| 23-26 | 4 | float | final_B | B 轮当前闭环反馈速度，单位 `m/s` |
| 27-30 | 4 | float | target_A | A 轮目标速度，单位 `m/s` |
| 31-34 | 4 | float | target_B | B 轮目标速度，单位 `m/s` |
| 35-36 | 2 | int16 | output_A | A 轮当前输出 PWM |
| 37-38 | 2 | int16 | output_B | B 轮当前输出 PWM |
| 39 | 1 | uint8 | checksum | XOR(bytes `0..38`) |

说明：
- 非 `AKM_CAR` 构建下，`t_raw_A/B` 与 `m_raw_A/B` 固定为 `0.0f`
- `final_A/B` 为当前实际进入速度环的反馈值

### 4.2 参数帧 `FrameID = 0x02`

- 总长度：`45` 字节
- 触发方式：收到查询命令 `0x30` 后，在下一个发送周期回复

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x02` |
| 3 | 1 | uint8 | ctrl_mode | 速度环模式，`0=PI`，`1=LADRC` |
| 4-7 | 4 | float | b0 | LADRC 控制增益估计 |
| 8-11 | 4 | float | wc | LADRC 闭环带宽 |
| 12-15 | 4 | float | wo | LADRC ESO 带宽 |
| 16-19 | 4 | float | kff | 速度前馈系数 |
| 20-23 | 4 | float | u_deadzone | 静摩擦补偿 PWM |
| 24-27 | 4 | float | rc_speed | 遥控速度基准 |
| 28-31 | 4 | float | limt_max_speed | 最大速度限制，单位 `m/s` |
| 32-35 | 4 | float | smooth_MotorStep | 速度平滑步进 |
| 36-39 | 4 | float | pi_kp | 当前 PI 的 `kp` |
| 40-43 | 4 | float | pi_ki | 当前 PI 的 `ki` |
| 44 | 1 | uint8 | checksum | XOR(bytes `0..43`) |

说明：
- 当前参数帧只回传 A 轮 PI 参数；B 轮通常与 A 轮保持一致
- `ctrl_mode` 对应 `robot_control.speed_ctrl_mode`

### 4.3 控制状态帧 `FrameID = 0x03`

- 总长度：`45` 字节
- 发送方式：仅在 `AKM_CAR` 构建下启用，与 `0x01` 数据帧交替发送

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | header1 | 固定 `0xAA` |
| 1 | 1 | uint8 | header2 | 固定 `0x55` |
| 2 | 1 | uint8 | frame_id | 固定 `0x03` |
| 3 | 1 | uint8 | ctrl_mode | 速度环模式，`0=PI`，`1=LADRC` |
| 4-7 | 4 | float | meas_A | A 轮当前闭环反馈速度 |
| 8-11 | 4 | float | meas_B | B 轮当前闭环反馈速度 |
| 12-15 | 4 | float | z1_A | A 轮 LADRC ESO 状态 `z1` |
| 16-19 | 4 | float | z1_B | B 轮 LADRC ESO 状态 `z1` |
| 20-23 | 4 | float | z2_A | A 轮 LADRC ESO 状态 `z2` |
| 24-27 | 4 | float | z2_B | B 轮 LADRC ESO 状态 `z2` |
| 28-31 | 4 | float | uff_A | A 轮上一拍前馈输出 |
| 32-35 | 4 | float | uff_B | B 轮上一拍前馈输出 |
| 36-37 | 2 | int16 | output_A | A 轮当前 PWM 输出 |
| 38-39 | 2 | int16 | output_B | B 轮当前 PWM 输出 |
| 40-41 | 2 | int16 | short_dt_A | A 轮 T 法异常短周期计数，饱和到 `32767` |
| 42-43 | 2 | int16 | short_dt_B | B 轮 T 法异常短周期计数，饱和到 `32767` |
| 44 | 1 | uint8 | checksum | XOR(bytes `0..43`) |

说明：
- `short_dt_A/B` 用于辅助判断编码器噪声或毛刺脉冲
- 当处于 `PI` 模式时，`z1/z2/uff` 仍可能保留最近一次 LADRC 状态

## 5. RX 命令定义

### 5.1 `0x10` 设置 A/B 两轮共同 PID

| 字段 | 值 |
|------|-----|
| CmdID | `0x10` |
| Length | `12` |
| Payload | `kp(float) + ki(float) + kd(float)` |

说明：
- 同时修改 `PI_MotorA`、`PI_MotorB`
- 同步更新全局 `robot.V_KP`、`robot.V_KI`

### 5.2 `0x11` 设置 A 轮 PID

| 字段 | 值 |
|------|-----|
| CmdID | `0x11` |
| Length | `12` |
| Payload | `kp(float) + ki(float) + kd(float)` |

### 5.3 `0x12` 设置 B 轮 PID

| 字段 | 值 |
|------|-----|
| CmdID | `0x12` |
| Length | `12` |
| Payload | `kp(float) + ki(float) + kd(float)` |

### 5.4 `0x20` 设置 `rc_speed`

| 字段 | 值 |
|------|-----|
| CmdID | `0x20` |
| Length | `4` |
| Payload | `value(float)` |
| 范围 | `0 < value <= 10000.0` |

### 5.5 `0x21` 设置 `limt_max_speed`

| 字段 | 值 |
|------|-----|
| CmdID | `0x21` |
| Length | `4` |
| Payload | `value(float)` |
| 范围 | `0 < value <= 10.0` |

### 5.6 `0x22` 设置 `smooth_MotorStep`

| 字段 | 值 |
|------|-----|
| CmdID | `0x22` |
| Length | `4` |
| Payload | `value(float)` |
| 范围 | `0 < value <= 1.0` |

### 5.7 `0x23` 设置 A/B 两轮目标速度

| 字段 | 值 |
|------|-----|
| CmdID | `0x23` |
| Length | `8` |
| Payload | `speed_a(float) + speed_b(float)` |
| 范围 | `0 <= value <= 10.0` |

说明：
- 进入 `UART_TARGET_MODE_SPEED`
- 当前实现对输入值取绝对值，方向仍由车体控制链路决定
- 切换到此模式时会清空之前的 UART PWM 目标状态并复位 A/B 控制器

### 5.8 `0x24` 设置 A/B 两轮目标 PWM

| 字段 | 值 |
|------|-----|
| CmdID | `0x24` |
| Length | `4` |
| Payload | `pwm_a(int16) + pwm_b(int16)` |
| 范围 | `0 <= value <= FULL_DUTYCYCLE` |

说明：
- 进入 `UART_TARGET_MODE_PWM`
- 当前实现对输入值取绝对值

### 5.9 `0x30` 查询当前参数

| 字段 | 值 |
|------|-----|
| CmdID | `0x30` |
| Length | `0` |
| Payload | 无 |

说明：
- 下一个发送周期会回传 `0x02` 参数帧

### 5.10 `0x40` 设置速度环模式

| 字段 | 值 |
|------|-----|
| CmdID | `0x40` |
| Length | `1` |
| Payload | `mode(uint8)` |
| 取值 | `0=PI`, `1=LADRC` |

说明：
- 切换模式时会复位 A/B 的 PI 与 LADRC 内部状态
- 切到 LADRC 时，`z1` 会先贴合当前 `robot.MOTOR_A/B.Encoder`

### 5.11 `0x41` 设置 A/B 两轮 LADRC 参数

| 字段 | 值 |
|------|-----|
| CmdID | `0x41` |
| Length | `20` |
| Payload | `b0(float) + wc(float) + wo(float) + kff(float) + u_deadzone(float)` |

有效范围：
- `0.00001 <= b0 <= 0.01`
- `1.0 <= wc <= 100.0`
- `1.0 <= wo <= 300.0`
- `0.0 <= kff <= 50000.0`
- `0.0 <= u_deadzone <= 5000.0`

说明：
- 同时更新 A/B 两轮 LADRC 参数
- 更新 `wc/wo` 后会立即重新计算 `beta1 = 2*wo`、`beta2 = wo*wo`

## 6. 当前默认值

AKM 目标下系统初始化默认切换到 LADRC，默认参数如下：

| 参数 | 默认值 |
|------|--------|
| `b0` | `0.00080` |
| `wc` | `18.0` |
| `wo` | `54.0` |
| `kff` | `7000.0` |
| `u_deadzone` | `900.0` |

速度环零速复位条件：
- `fabs(target) < 0.01`
- 且 `fabs(meas) < 0.02`

## 7. 校验算法

发送和接收均使用逐字节 XOR 校验。

示例：

```c
uint8_t checksum = 0;
for (i = 0; i < len_without_checksum; i++)
{
    checksum ^= buf[i];
}
```

## 8. 注意事项

- 当前协议主要围绕 `AKM_CAR` 调试设计；其他底盘仍可使用基础参数命令，但 `0x03` 控制状态帧的 LADRC 含义仅对 AKM A/B 两轮成立。
- `0x01` 与 `0x03` 在 AKM 模式下交替发送，所以单个帧 ID 的实际刷新率约为 `50Hz`。
- 运行环境中没有参数持久化逻辑；通过调试串口下发的 PID/LADRC 参数重启后不会自动保存。
- `SET_SPEED_AB` 与 `SET_PWM_AB` 互斥，后下发的模式会覆盖前一种模式。
- 文档应以源码为准；若后续修改了 `DEBUG_*` 宏、帧长度或字段顺序，需要同步更新本文。
