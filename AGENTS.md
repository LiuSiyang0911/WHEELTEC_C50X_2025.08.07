# AGENTS.md

AI 编码代理在此仓库中工作的指引。

## 项目概述

WHEELTEC C50X 阿克曼（Ackermann）底盘机器人固件，目标芯片 **STM32F407VE**（Cortex-M4），运行 **FreeRTOS**。

语言：C。源码中中英双语注释，文件编码 GB2312/GBK。

**仅关注 `AKM_CAR`（阿克曼车型）**，其他车型（差速/麦轮/四驱/全向）存在于条件编译中，无需处理。

## 构建系统

**IDE**：Keil µVision 5（ARM Compiler v5.06）
**工程文件**：`USER/WHEELTEC.uvprojx`
**输出目录**：`OBJ/`

**无 Makefile、无命令行构建工具**。不要尝试 `make`、`cmake`、`gcc` 等命令。

### 构建目标

选择 Keil 目标 `Akm_Car`，预处理器定义 `AKM_CAR`。

所有目标通用定义：`STM32F40_41xxx, USE_STDPERIPH_DRIVER, __FPU_PRESENT=1, __TARGET_FPU_VFP, ARM_MATH_CM4, __CC_ARM, USE_USB_OTG_FS`

### 测试与代码检查

本项目**无自动化测试、无 linter、无类型检查**。验证方式为硬件实测。修改代码后确保 Keil 编译无错误无警告。

## 目录结构

| 目录 | 说明 |
|------|------|
| `USER/` | 入口 `main.c`、系统初始化 `system.c`/`system.h`、Keil 工程文件 |
| `BALANCE/` | FreeRTOS 任务实现（运动控制、数据输出、显示、LED、IMU） |
| `BALANCE/Inc/` | 任务文件头文件和控制结构体定义 |
| `CarType/` | 底盘参数初始化（轮距、轴距、减速比、编码器配置） |
| `CarType/Inc/` | 机器人配置结构体 `robot_init.h` |
| `HARDWARE/` | 外设驱动（电机、编码器、UART、CAN、I2C、ADC、OLED、USB 等） |
| `HARDWARE/Inc/` | 外设驱动头文件 |
| `HARDWARE/MPU6050/` | MPU6050 驱动（V1.0 硬件） |
| `HARDWARE/ICM20948/` | ICM20948 驱动（V1.1 硬件） |
| `FWLIB/` | STM32F4xx 标准外设库（厂商代码，勿改） |
| `FreeRTOS/` | FreeRTOS 内核（厂商代码，勿改） |
| `CORE/` | ARM CMSIS 头文件（厂商代码） |
| `MiddleWares/` | USB Host 中间件（厂商代码） |
| `USB_HOST/` | USB 主机应用层（手柄适配） |
| `SYSTEM/` | 底层系统初始化（时钟、延时、Flash） |

## FreeRTOS 任务

所有任务在 `USER/main.c` → `start_task()` 中创建：

| 任务 | 文件 | 优先级 | 频率 | 用途 |
|------|------|--------|------|------|
| Balance_task | `BALANCE/balance_task.c` | 5 | 100Hz | 核心运动控制、PI 调节、运动学 |
| data_task | `BALANCE/data_task.c` | 4 | 20Hz | UART3/CAN 遥测发送 |
| MPU6050_task / ICM20948_task | `BALANCE/imu_task.c` | 3 | 100Hz | IMU 数据读取 |
| show_task | `BALANCE/show_task.c` | - | - | OLED 显示 |
| led_task | `BALANCE/led_task.c` | - | - | LED 状态灯 |
| pstwo_task | `BALANCE/ps2_task.c` | - | - | PS2 手柄（仅 V1.0） |

频率宏定义在 `USER/system.h`：`RATE_1_HZ` ~ `RATE_1000_HZ`。
周期任务使用 `vTaskDelayUntil(&lastWakeTime, F2T(RATE_xxx_HZ))`。

## 硬件版本

运行时通过 `SysVal.HardWare_Ver` 检测（`USER/system.c`）：

- **V1_0**：MPU6050 IMU，CAN 在 PA11/PA12，支持 PS2 手柄
- **V1_1**：ICM20948 IMU，CAN 在 PD0/PD1，USB 手柄

硬件相关代码用 `if(SysVal.HardWare_Ver == V1_0)` 或 `== V1_1` 保护。

## AKM 专属架构

阿克曼车型与其他车型的关键区别：

- **驱动**：2 电机（Motor_A 左后轮、Motor_B 右后轮）+ 1 舵机（前轮转向）
- **编码器**：T 法测速（EXTI 边沿检测 + TIM6 自由运行计数器），仅 A/B 两路
- **运动学**：`InverseKinematics_akm(Vx, Vz)` 将线速度+角速度转换为左右轮速+舵机角度
- **舵机**：非自锁检测 `Servo_UnLock_Check`，结构体 `AKM_SERVO_UNLOCK_t`
- **子型号**：10 种硬件子型号，通过 ADC 电位器读取选择（`CarType/akm_robot_init.c`）
- **ADC2**：仅 AKM 初始化，用于舵机滑动变阻器反馈
- **Encoder C/D 不初始化**

## 关键数据结构

`CarType/Inc/robot_init.h`：
- `ROBOT_t` — 机器人参数、电机状态、硬件参数
- `Robot_Parament_InitTypeDef` — 轮距、轴距、减速比、编码器精度
- `Moto_parameter` — 单电机的目标值、编码器反馈、PWM 输出

`BALANCE/Inc/balance_task.h`：
- `ROBOT_CONTROL_t` — 目标速度 (Vx, Vy, Vz)、控制模式、平滑参数
- `PI_CONTROLLER` — 增量式 PI 调节器
- `AKM_SERVO_UNLOCK_t` — AKM 舵机非自锁控制
- `SEND_DATA` — 24 字节遥测帧（帧头 0x7B，帧尾 0x7D）

## 编码规范

### 文件编码
源文件使用 **GB2312/GBK 编码**存放中文注释，不要转换为 UTF-8。

### 头文件保护
`#ifndef __NAME_H` / `#define __NAME_H`（双下划线前缀）。

### Include 顺序
1. `"system.h"`（主包含文件，引入 STM32 SPL、FreeRTOS、所有硬件驱动）
2. 模块特定头文件

大多数 `.c` 文件只包含自己的 `.h`，而 `.h` 再包含 `system.h`。

### 命名约定
- **类型/结构体**：PascalCase + `_t` 后缀 — `ROBOT_CONTROL_t`、`PI_CONTROLLER`
- **宏/常量**：全大写 — `RATE_100_HZ`、`BALANCE_TASK_PRIO`
- **函数**：PascalCase 或混合 — `Balance_task`、`Drive_Motor`、`PI_Controller_Init`
- **变量**：混合 — `robot_control`、`PI_MotorA`、`ServoState`
- **电机编号**：`Motor_A`(1)、`Motor_B`(2)
- **内部函数**：在头文件中声明为 `static` 以供文档说明

### 注释风格
- 双语：英文注释后跟中文注释
- 函数块使用 `/**** ... ****/` 横幅格式，包含 Function/Input/Output
- 行内注释用 `//`

### 条件编译
AKM 专用代码用 `#if defined AKM_CAR` 包裹。在 `balance_task.c`、`data_task.c`、`show_task.c` 等文件中大量使用。

### FreeRTOS 惯例
- 任务函数返回 `void`，参数 `void *pvParameters`
- 临界区用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`
- 周期任务用 `vTaskDelayUntil()` + `F2T()` 宏

### STM32 外设库
使用标准外设库函数（`GPIO_Init`、`TIM_TimeBaseInit` 等），不使用 HAL 或 LL 驱动。

## 通信接口

- **UART1**（PA9/PA10）— 调试输出，115200 波特率
- **UART3**（PB10/PB11 或 PD8/PD9）— ROS 通信，115200，主要遥测口
- **UART4**（PC10/PC11 或 PA0/PA1）— 蓝牙 APP，9600 波特率
- **CAN1** — 控制和遥测（引脚映射随硬件版本变化）
- **USB Host** — Xbox 360、PS2、HID 手柄

遥测帧：24 字节，帧头 `0x7B`，帧尾 `0x7D`，BCC 校验。

## 约束

- **不要修改** `FWLIB/`、`FreeRTOS/`、`CORE/`、`MiddleWares/`（厂商代码）
- 只能定义一个底盘类型 `AKM_CAR`
- 硬件专用代码必须检查 `SysVal.HardWare_Ver`
- 电机输出通过 TIM8 PWM（10kHz）+ 方向 GPIO
- 编码器反馈 AKM 使用 T 法（EXTI + TIM6），非正交模式
