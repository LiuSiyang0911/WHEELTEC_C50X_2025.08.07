# WHEELTEC C50X 蓝牙APP通信协议

## 目录
- [1. 概述](#1-概述)
- [2. 硬件配置](#2-硬件配置)
- [3. 通信协议格式](#3-通信协议格式)
- [4. 下行协议：APP → Robot（控制命令）](#4-下行协议app--robot控制命令)
- [5. 上行协议：Robot → APP（遥测数据）](#5-上行协议robot--app遥测数据)
- [6. 参数配置帧](#6-参数配置帧)
- [7. APP界面按键映射](#7-app界面按键映射)
- [8. 速度控制流程](#8-速度控制流程)
- [9. 蓝牙AT指令过滤](#9-蓝牙at指令过滤)
- [10. 数据结构定义](#10-数据结构定义)
- [11. 源码位置索引](#11-源码位置索引)

---

## 1. 概述

WHEELTEC C50X 阿克曼底盘通过蓝牙模块与手机APP进行通信，实现遥控驾驶和参数调试功能。

**通信方向：**
- **下行（APP → Robot）：** 运动控制命令、参数配置
- **上行（Robot → APP）：** 实时遥测数据（轮速、陀螺仪、电池电压）

**主要功能：**
- 八方向摇杆遥控（前进、后退、左转、右转及组合方向）
- 速度加减调节
- 实时遥测显示（首页数据 + 波形数据）
- PID参数在线调试、参数保存至Flash
- 自动充电模式切换

**通信接口：** UART4，波特率 9600

---

## 2. 硬件配置

### 2.1 UART4引脚配置

| 引脚 | GPIO | 功能 |
|------|------|------|
| TX | PC10 | 发送 |
| RX | PC11 | 接收 |

### 2.2 串口参数

| 参数 | 值 |
|------|-----|
| 波特率 | 9600 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 流控 | 无 |

### 2.3 初始化代码位置

```
文件: HARDWARE/uartx.c
函数: UART4_Init(u32 bound)
行号: 178-232
```

```
文件: USER/system.c
行号: 125-129
代码: UART4_Init(9600);  // 正常模式
```

---

## 3. 通信协议格式

蓝牙APP与STM32之间采用**双向通信**，通过UART4（9600bps）传输。

### 3.1 协议类型总览

**下行协议（APP → Robot）：**

| 类型 | 用途 | 格式 |
|------|------|------|
| 单字节控制 | 实时运动控制、界面切换 | 1字节指令 |
| 参数配置帧 | PID调试、参数读写 | 0x7B...0x7D 包裹 |

**上行协议（Robot → APP）：**

| 类型 | 用途 | 格式 |
|------|------|------|
| A帧 | 首页数据（轮速、电压、陀螺仪Z） | `{A左:右:电压:陀螺Z}$` |
| B帧 | 波形数据（陀螺仪XYZ） | `{BX:Y:Z}$` |
| C帧 | 调试参数回读 | `{C9个参数}$` |

### 3.2 printf重定向

所有上行数据通过 `printf` 发送，`fputc` 已重定向到 UART4：

```c
// 文件: SYSTEM/usart/usart.c (行26-30)
int fputc(int ch, FILE *f)
{
    while((UART4->SR&0X40)==0);
    UART4->DR = (u8) ch;
    return ch;
}
```

---

## 4. 下行协议：APP → Robot（控制命令）

### 4.1 方向控制命令（摇杆界面）

APP摇杆界面发送ASCII字母或等效二进制值：

| 命令 | ASCII | 十六进制 | 二进制值 | 方向 |
|------|-------|----------|----------|------|
| A | 'A' | 0x41 | 0x01 | 前进 |
| B | 'B' | 0x42 | 0x02 | 前进+右转 |
| C | 'C' | 0x43 | 0x03 | 右转 |
| D | 'D' | 0x44 | 0x04 | 后退+右转 |
| E | 'E' | 0x45 | 0x05 | 后退 |
| F | 'F' | 0x46 | 0x06 | 后退+左转 |
| G | 'G' | 0x47 | 0x07 | 左转 |
| H | 'H' | 0x48 | 0x08 | 前进+左转 |
| 松开 | - | - | 0x00 | 停止 |

**方向示意图：**
```
        A (前进)
        ^
   H <--+--> B
   |    |    |
   +----+----+--->  (Y轴负方向)
   |    |    |
   G <--+--> C
        |
   F <--+--> D
        v
        E (后退)
```

### 4.2 速度调节命令

| 命令 | 十六进制 | 功能 |
|------|----------|------|
| X | 0x58 | 加速 +100 mm/s |
| Y | 0x59 | 减速 -100 mm/s |

**速度限制：** 受 `robot_control.limt_max_speed` 限制（默认 3.5 m/s，即 3500 mm/s）

### 4.3 界面切换命令

| 命令 | 十六进制 | 功能 |
|------|----------|------|
| K | 0x4B | 进入舵机转向控制界面 |
| I | 0x49 | 返回摇杆控制界面 |
| J | 0x4A | 返回摇杆控制界面 |

### 4.4 舵机转向控制命令（转向界面）

| 命令 | 十六进制 | 功能 |
|------|----------|------|
| C | 0x43 | 右转 |
| G | 0x47 | 左转 |
| A | 0x41 | 前进 |
| E | 0x45 | 后退 |

### 4.5 功能控制命令

| 命令 | ASCII | 功能 |
|------|-------|------|
| b | 'b' (0x62) | 切换自动充电使能 |
| m | 'm' (0x6D) | 切换OLED显示页面 |

### 4.6 APP控制模式激活

APP按下前进键（0x41）**连续两次**可激活APP控制模式：

```c
if(Usart_Receive==0x41 && Last_Usart_Receive==0x41 && (Get_Control_Mode(_APP_Control))==0)
    Set_Control_Mode(_APP_Control);
```

**代码位置：** `BALANCE/uartx_callback.c:178-179`

---

## 5. 上行协议：Robot → APP（遥测数据）

Robot通过 `printf`（重定向到UART4）向APP发送实时遥测数据，由 `show_task` 任务以 **10Hz** 频率发送。

### 5.1 数据发送机制

```c
// 文件: BALANCE/show_task.c (行77-141)
static void APP_ShowTask(void)
{
    // A帧和B帧交替发送，各5Hz
    flag_show = !flag_show;

    if(appkey.ParamSendflag == 1) {
        // C帧：调试参数（APP请求时发送一次）
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", ...);
        appkey.ParamSendflag = 0;
    }
    else if(flag_show == 0) {
        // A帧：首页数据
        printf("{A%d:%d:%d:%d}$", Left_Figure, Right_Figure, Voltage_Show, gyro_z);
    }
    else {
        // B帧：波形数据
        printf("{B%d:%d:%d}$", gyro_x, gyro_y, gyro_z);
    }
}
```

### 5.2 A帧 — 首页数据

**格式：** `{A左轮速:右轮速:电池电压:陀螺仪Z}$`

**发送频率：** 5Hz（与B帧交替）

| 字段 | 数据来源 | 单位 | 说明 |
|------|----------|------|------|
| 左轮速 | `robot.MOTOR_A.Encoder × 100` | 0.01 m/s | 取绝对值 |
| 右轮速 | `robot.MOTOR_B.Encoder × 100` | 0.01 m/s | 取绝对值 |
| 电池电压 | `(voltage×100-2000)×5/26` | % (0-100) | 电压百分比 |
| 陀螺仪Z | `imu.gyro.z` | °/s | 原始值 |

**示例：** `{A150:165:85:320}$`

### 5.3 B帧 — 波形数据

**格式：** `{B陀螺X:陀螺Y:陀螺Z}$`

**发送频率：** 5Hz（与A帧交替）

| 字段 | 数据来源 | 单位 |
|------|----------|------|
| 陀螺X | `imu.gyro.x` | °/s |
| 陀螺Y | `imu.gyro.y` | °/s |
| 陀螺Z | `imu.gyro.z` | °/s |

**示例：** `{B12:-45:320}$`

### 5.4 C帧 — 调试参数回读

**格式：** `{Crc_speed:Kp:Ki:smooth_Motor:smooth_Servo:ServoMax:ServoMin:ServoMid:LineDiff}$`

**触发条件：** APP发送参数读取请求（`Receive[3]==0x50`）后发送一次

| 字段 | 数据来源 | 说明 |
|------|----------|------|
| rc_speed | `(int)robot_control.rc_speed` | 遥控基准速度 (mm/s) |
| Kp | `(int)robot.V_KP` | PI比例系数 |
| Ki | `(int)robot.V_KI` | PI积分系数 |
| smooth_Motor | `(int)(smooth_MotorStep×1000)` | 电机平滑系数×1000 |
| smooth_Servo | `(int)smooth_ServoStep` | 舵机平滑系数 |
| ServoMax | `(int)Akm_Servo.Max` | 舵机PWM最大值 |
| ServoMin | `(int)Akm_Servo.Min` | 舵机PWM最小值 |
| ServoMid | `(int)Akm_Servo.Mid` | 舵机PWM中值 |
| LineDiff | `robot_control.LineDiffParam` | 偏航系数 (0-100) |

**示例：** `{C500:800:200:50:100:1850:550:1200:0}$`

### 5.5 上行数据流向

```
┌─────────────┐   printf()   ┌──────────┐   UART4 TX   ┌──────────┐
│ show_task   │ ──────────> │  fputc()  │ ──────────> │ 蓝牙APP  │
│ (10Hz)      │  A/B/C帧    │ (重定向)  │   9600bps   │ (手机)   │
└─────────────┘             └──────────┘             └──────────┘
```

**注意：** 24字节二进制遥测帧（速度+IMU+电压）仅通过 UART3（ROS）和 CAN1 发送，**不走蓝牙通道**。

**代码位置：** `BALANCE/show_task.c:77-141`，`SYSTEM/usart/usart.c:26-30`

---

## 6. 参数配置帧

### 6.1 帧格式

```
+--------+--------+--------+--------+-----+--------+
| 0x7B   | Cmd    | Index  | Data   | ... | 0x7D   |
| 帧头   | 类型   | 参数号 | 数值   |     | 帧尾   |
+--------+--------+--------+--------+-----+--------+
```

| 字段 | 值 | 说明 |
|------|-----|------|
| 帧头 | 0x7B | 固定起始标识 |
| 帧尾 | 0x7D | 固定结束标识 |
| 数据 | ASCII数字 | 参数值（十进制ASCII编码） |

### 6.2 单参数配置命令

当 `Receive[1] != 0x23` 时，按单参数解析：

| Receive[1] | 十六进制 | 参数名称 | 说明 |
|------------|----------|----------|------|
| 0x30 | '0' | rc_speed | 遥控基准速度 (mm/s) |
| 0x31 | '1' | Kp | PI控制器比例系数 |
| 0x32 | '2' | Ki | PI控制器积分系数 |
| 0x33 | '3' | smooth_MotorStep | 电机速度平滑系数 (/1000) |
| 0x34 | '4' | smooth_ServoStep | 舵机速度平滑系数 |
| 0x35 | '5' | Akm_Servo.Max | 舵机PWM最大值 |
| 0x36 | '6' | Akm_Servo.Min | 舵机PWM最小值 |
| 0x37 | '7' | Akm_Servo.Mid | 舵机PWM中值 |
| 0x38 | '8' | LineDiffParam | 偏航系数 (0-100) |

### 6.3 批量参数配置命令

当 `Receive[1] == 0x23` (`'#'`) 时，解析批量参数字符串：

**格式：** `#num1:num2:num3:num4:num5:num6:num7:num8:num9}`

| 索引 | 参数 | 范围/单位 |
|------|------|-----------|
| dataArray[0] | rc_speed | mm/s |
| dataArray[1] | Kp | - |
| dataArray[2] | Ki | - |
| dataArray[3] | Kd / smooth_MotorStep | /1000 |
| dataArray[4] | smooth_ServoStep | - |
| dataArray[5] | Akm_Servo.Max | PWM值 |
| dataArray[6] | Akm_Servo.Min | PWM值 |
| dataArray[7] | Akm_Servo.Mid | PWM值 |
| dataArray[8] | LineDiffParam | 0-100 |

**解析逻辑：** 以 `:` 分隔数字，ASCII字符 `'0'-'9'` 转换为数值。

### 6.4 特殊指令

| Receive[3] | 十六进制 | 功能 |
|------------|----------|------|
| 0x50 | 'P' | 读取参数：发送当前参数到APP显示 |
| 0x57 | 'W' | 写入参数：保存参数到STM32 Flash |

**代码位置：** `BALANCE/uartx_callback.c:221-323`

---

## 7. APP界面按键映射

### 7.1 摇杆控制界面（TurnPage = 0）

```
┌─────────────────────────────────┐
│  [A]↑  [B]↗  [加速X]  [减速Y]  │
│  [H]←  ○    [B]→              │
│  [G]↙  [F]↓  [E]↘            │
│                                 │
│  [K]进入转向界面                │
└─────────────────────────────────┘
```

### 7.2 舵机转向控制界面（TurnPage = 1）

```
┌─────────────────────────────────┐
│  [A]前进  [E]后退              │
│  [G]左转  [C]右转              │
│                                 │
│  [I]/[J]返回摇杆界面           │
└─────────────────────────────────┘
```

### 7.3 调试参数界面

通过发送 `0x7B...0x7D` 帧进行参数读写：
- 发送 `0x50` 在 Receive[3]：请求APP显示当前参数
- 发送 `0x57` 在 Receive[3]：保存参数到Flash

---

## 8. 速度控制流程

### 8.1 下行数据流

```
┌──────────┐    UART4    ┌──────────────┐    ┌─────────────┐
│ 蓝牙APP  │ ─────────> │ UART4_IRQHandler │ ──> │ Get_APPcmd  │
│ (手机)   │   9600bps  │ (中断处理)    │    │ (速度计算)  │
└──────────┘            └──────────────┘    └──────┬──────┘
                                                   │
                                                   v
                                            ┌─────────────┐
                                            │ Drive_Motor │
                                            │ (运动学解算) │
                                            └──────┬──────┘
                                                   │
                                                   v
                                            ┌─────────────┐
                                            │ 电机PWM输出 │
                                            └─────────────┘
```

### 8.2 方向-速度映射（阿克曼车型）

**基准速度：** `robot_control.rc_speed`（默认 500 mm/s）

**基准角速度：** `base_vz = angle_to_rad(30)`（30度转弧度）

| 方向 | Vx (mm/s) | Vz (rad) | 说明 |
|------|-----------|----------|------|
| A (1) | +rc_speed | 0 | 直行前进 |
| B (2) | +rc_speed | -base_vz | 前进+右转 |
| C (3) | 0 | -base_vz | 原地右转 |
| D (4) | -rc_speed | +base_vz | 后退+左转 |
| E (5) | -rc_speed | 0 | 直行后退 |
| F (6) | -rc_speed | -base_vz | 后退+右转 |
| G (7) | 0 | +base_vz | 原地左转 |
| H (8) | +rc_speed | +base_vz | 前进+左转 |

**单位转换（1292-1294行）：**
```c
robot_control.Vx = robot_control.Vx / 1000.0f;  // mm/s -> m/s
robot_control.Vz = robot_control.Vz * (robot_control.rc_speed / 500.0f);
```

### 8.3 Z轴速度缩放

Vz实际输出值会根据当前速度档位缩放：
```
实际Vz = base_vz × (rc_speed / 500.0)
```

即速度越快，转向角度对应的角速度越大。

**代码位置：** `BALANCE/balance_task.c:1225-1299`

---

## 9. 蓝牙AT指令过滤

蓝牙模块在连接/断开时会发送AT指令字符串，需要过滤以防止干扰正常控制。

### 9.1 过滤的AT指令

**断开时发送（34字节）：**
```
+DISC:SUCCESS\r\n
+READY\r\n
+PAIRABLE\r\n
```

**连接时发送（44字节，含MAC地址）：**
```
+CONNECTING<<XX:XX:XX:XX:XX:XX\r\n
+CONNECTED\r\n
```

### 9.2 过滤逻辑

当收到 `'+'` 字符时开始追踪，逐字符匹配已知AT指令模式：
- 匹配成功：返回1（丢弃该字节）
- 匹配失败：重置状态，返回0（正常处理）

**代码位置：** `BALANCE/uartx_callback.c:384-453`

---

## 10. 数据结构定义

### 10.1 APP控制标志结构

```c
// 文件: HARDWARE/Inc/uartx.h (行6-12)
typedef struct{
    u8 TurnPage;      // APP转向页面标志位 (0:摇杆, 1:转向)
    u8 DirectionFlag; // APP方向标志 (1-8 对应A-H)
    u8 ParamSaveFlag; // APP保存参数标志位
    u8 ParamSendflag; // APP发送数据标志位
    u8 TurnFlag;      // APP转向标志 (0:无, 1:左, 2:右)
} APP_CONTROL_t;
```

### 10.2 机器人控制结构（部分）

```c
// 文件: BALANCE/Inc/balance_task.h (行13-35)
typedef struct{
    u8 ControlMode;       // 控制模式位掩码
    float Vx;             // X轴目标速度 (m/s)
    float Vy;             // Y轴目标速度 (m/s)
    float Vz;             // Z轴目标速度 (rad/s)
    float rc_speed;       // 遥控基准速度 (mm/s, 默认500)
    float limt_max_speed; // 最大速度限制 (m/s, 默认3.5)
    float smooth_MotorStep;  // 电机速度平滑系数
    float smooth_ServoStep;  // 舵机速度平滑系数
    uint32_t LineDiffParam;  // 偏航系数 (0-100)
} ROBOT_CONTROL_t;
```

### 10.3 控制模式定义

```c
// 文件: BALANCE/Inc/balance_task.h (行87-95)
enum{
    _ROS_Control   = (1<<0),  // ROS控制
    _PS2_Control   = (1<<1),  // PS2手柄控制
    _APP_Control   = (1<<2),  // APP控制
    _RC_Control    = (1<<3),  // 航模遥控控制
    _CAN_Control   = (1<<4),  // CAN通信控制
    _USART_Control = (1<<5),  // 串口控制
};
```

---

## 11. 源码位置索引

| 功能模块 | 文件路径 | 关键行号 |
|----------|----------|----------|
| **--- 下行协议 ---** | | |
| UART4初始化 | `HARDWARE/uartx.c` | 178-232 |
| UART4波特率选择 | `USER/system.c` | 125-129 |
| UART4中断处理 | `BALANCE/uartx_callback.c` | 154-332 |
| AT指令过滤 | `BALANCE/uartx_callback.c` | 384-453 |
| 系统复位命令 | `BALANCE/uartx_callback.c` | 458-479 |
| APP控制结构定义 | `HARDWARE/Inc/uartx.h` | 6-12 |
| APP参数初始化 | `HARDWARE/uartx.c` | 6-13 |
| APP命令处理(速度) | `BALANCE/balance_task.c` | 1225-1299 |
| APP控制模式调用 | `BALANCE/balance_task.c` | 133 |
| 参数帧解析 | `BALANCE/uartx_callback.c` | 221-323 |
| **--- 上行协议 ---** | | |
| printf重定向 | `SYSTEM/usart/usart.c` | 26-30 |
| 上行数据发送任务 | `BALANCE/show_task.c` | 77-141 |
| 遥测任务周期 | `BALANCE/Inc/show_task.h` | 7 |
| 24字节帧组包(ROS/CAN) | `BALANCE/data_task.c` | 100-163 |
| BCC校验函数 | `BALANCE/data_task.c` | 46-52 |
| **--- 数据结构 ---** | | |
| 帧头帧尾定义 | `BALANCE/Inc/data_task.h` | 12-13 |
| 控制模式定义 | `BALANCE/Inc/balance_task.h` | 87-95 |
| 控制结构体定义 | `BALANCE/Inc/balance_task.h` | 13-35 |

---

## 附录：完整命令速查表

### A. 下行控制命令（APP → Robot）

| 命令 | 值 | 功能 |
|------|-----|------|
| A | 0x41 / 0x01 | 前进 |
| B | 0x42 / 0x02 | 前进+右转 |
| C | 0x43 / 0x03 | 右转 |
| D | 0x44 / 0x04 | 后退+右转 |
| E | 0x45 / 0x05 | 后退 |
| F | 0x46 / 0x06 | 后退+左转 |
| G | 0x47 / 0x07 | 左转 |
| H | 0x48 / 0x08 | 前进+左转 |
| X | 0x58 | 加速 +100mm/s |
| Y | 0x59 | 减速 -100mm/s |
| K | 0x4B | 进入转向界面 |
| I | 0x49 | 返回摇杆界面 |
| J | 0x4A | 返回摇杆界面 |
| b | 0x62 | 切换充电使能 |
| m | 0x6D | 切换OLED页面 |

### B. 参数配置命令（APP → Robot）

| 帧内容 | 功能 |
|--------|------|
| `{0x30 + 数据 + 0x7D}` | 设置遥控速度 |
| `{0x31 + 数据 + 0x7D}` | 设置Kp |
| `{0x32 + 数据 + 0x7D}` | 设置Ki |
| `{0x33 + 数据 + 0x7D}` | 设置平滑系数 |
| `{0x34 + 数据 + 0x7D}` | 设置舵机平滑 |
| `{0x35 + 数据 + 0x7D}` | 设置舵机Max |
| `{0x36 + 数据 + 0x7D}` | 设置舵机Min |
| `{0x37 + 数据 + 0x7D}` | 设置舵机Mid |
| `{0x38 + 数据 + 0x7D}` | 设置偏航系数 |
| `{...0x50...0x7D}` | 读取参数到APP |
| `{...0x57...0x7D}` | 保存参数到Flash |
| `{#d1:d2:...:d9}` | 批量设置参数 |

### C. 上行遥测帧（Robot → APP）

| 帧类型 | 格式 | 触发条件 | 示例 |
|--------|------|----------|------|
| A帧 | `{A左轮速:右轮速:电压%:陀螺Z}` | 10Hz（交替5Hz） | `{A150:165:85:320}` |
| B帧 | `{B陀螺X:陀螺Y:陀螺Z}` | 10Hz（交替5Hz） | `{B12:-45:320}` |
| C帧 | `{C9个参数}` | APP请求后发送1次 | `{C500:800:200:50:100:1850:550:1200:0}` |

---

*文档版本: v1.1（新增上行遥测协议）*
*适用固件: WHEELTEC C50X 2025.08.07*
*目标芯片: STM32F407VE*
*车型: AKM_CAR (阿克曼)*
