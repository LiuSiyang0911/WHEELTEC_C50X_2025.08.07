# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WHEELTEC C50X multi-chassis robot firmware. Runs on **STM32F407VE** (Cortex-M4) with **FreeRTOS**. Supports 5 robot chassis types (Ackermann, Differential, Mecanum, 4WD, Omnidirectional) with 48+ hardware variants selected at runtime via ADC potentiometer.

Language: C (bilingual comments — English with Chinese translations throughout).

## Build System

**IDE**: Keil µVision 5 (ARM Compiler v5.06)
**Project file**: `USER/WHEELTEC.uvprojx`
**Output**: `OBJ/` directory (hex, bin, map, axf files)

**No Makefile, no CLI build.** Do not attempt `make`, `cmake`, `gcc`, etc. Build from Keil: select target → Build (F7).

**No automated tests, no linter, no type checker.** Validation is hardware testing only. Ensure Keil compiles with zero errors and zero warnings.

### Build Targets

Each chassis type is a separate Keil target with its own preprocessor define:

| Target | Define | Output |
|--------|--------|--------|
| Akm_Car | `AKM_CAR` | Akm_Car.hex |
| Diff_Car | `DIFF_CAR` | Diff_Car.hex |
| Mec_Car | `MEC_CAR` | Mec_Car.hex |
| 4WD_Car | `_4WD_CAR` | 4WD_Car.hex |
| Omni_Car | `OMNI_CAR` | Omni_Car.hex |

Common defines across all targets: `STM32F40_41xxx, USE_STDPERIPH_DRIVER, __FPU_PRESENT=1, __TARGET_FPU_VFP, ARM_MATH_CM4, __CC_ARM, USE_USB_OTG_FS`

Build hooks: `hexbinHandle.bat` (before), `fromelf` + `copyhex.bat` (after — converts to BIN).

## Architecture

### Directory Layout

- **USER/** — Entry point (`main.c`), system init (`system.c`), master include (`system.h`), Keil project files
- **BALANCE/** — FreeRTOS task implementations (motion control, data output, display, LED, IMU)
- **BALANCE/Inc/** — Headers for task files and control structures
- **CarType/** — Per-chassis-type robot parameter initialization (wheel spacing, gear ratio, encoder config)
- **CarType/Inc/** — Robot config structures (`robot_init.h`)
- **HARDWARE/** — Peripheral drivers (motor, encoder, UART, CAN, I2C, ADC, OLED, USB host, etc.)
- **HARDWARE/Inc/** — Hardware driver headers
- **HARDWARE/MPU6050/** — MPU6050 IMU driver (V1.0 hardware)
- **HARDWARE/ICM20948/** — ICM20948 IMU driver (V1.1 hardware)
- **FWLIB/** — STM32F4xx Standard Peripheral Library (**vendor code, do not modify**)
- **FreeRTOS/** — FreeRTOS kernel (**vendor code, do not modify**)
- **CORE/** — ARM CMSIS headers (**vendor code, do not modify**)
- **MiddleWares/** — USB Host middleware library (**vendor code, do not modify**)
- **USB_HOST/** — USB host application layer (gamepad adapters)
- **SYSTEM/** — Low-level system init (clocks, delay, flash)
- **docs/** — Protocol documentation (Bluetooth APP protocol)

### Control Flow

```
main() → systemInit() → xTaskCreate(start_task) → vTaskStartScheduler()

Input Sources (UART3/CAN/UART4/PS2/USB/RC)
    ↓ UartxControll_Callback / CAN1_RX0_IRQHandler
    ↓ Writes to: robot_control.{Vx, Vy, Vz, Mode}
    ↓
Balance_task (100Hz):
    Get_Robot_FeedBack()        ← Read encoder speeds + servo position
    Robot_SelfCheck()           ← Startup validation (first 10s)
    ResponseControl()           ← Command loss timeout handling
    Drive_Motor(Vx, Vy, Vz)    ← Inverse kinematics → per-motor targets
    Incremental_MOTOR(PI, ...)  ← PI controller: target vs encoder → PWM
    Set_Pwm(A, B, C, D, servo) ← Output to TIM8 PWM + direction GPIO
    ↓
data_task (20Hz):
    Kinematics_*()              ← Forward kinematics: encoder → Vx, Vy, Vz
    Transmit 24-byte telemetry  → UART3/CAN
```

### FreeRTOS Tasks

All tasks created in `USER/main.c` → `start_task()`:

| Task | File | Priority | Rate | Purpose |
|------|------|----------|------|---------|
| Balance_task | `BALANCE/balance_task.c` | 5 | 100Hz | Core motion control, PI regulators, kinematics |
| data_task | `BALANCE/data_task.c` | 4 | 20Hz | Telemetry TX on UART3/CAN |
| MPU6050_task / ICM20948_task | `BALANCE/imu_task.c` | 3 | 100Hz | IMU sensor read + zero calibration |
| show_task | `BALANCE/show_task.c` | 2 | 10Hz | OLED display + APP status |
| led_task | `BALANCE/led_task.c` | 2 | - | Status LEDs |
| pstwo_task | `BALANCE/ps2_task.c` | - | - | PS2 controller (V1.0 hardware only) |

Timing macros in `USER/system.h`: `RATE_1_HZ` through `RATE_1000_HZ`, used with `F2T(X)` macro (`configTICK_RATE_HZ / X` — converts frequency to FreeRTOS ticks). Periodic tasks use `vTaskDelayUntil(&lastWakeTime, F2T(RATE_xxx_HZ))`.

### Hardware Version Detection

Two hardware revisions detected at runtime in `USER/system.c` via IMU device ID probe:
- **V1_0**: MPU6050 IMU, CAN on PA11/PA12, PS2 controller support
- **V1_1**: ICM20948 IMU, CAN on PD0/PD1, USB gamepad support, no PS2

All hardware-specific code must check `SysVal.HardWare_Ver`.

### Key Data Structures

**`CarType/Inc/robot_init.h`:**
- `ROBOT_t` — Complete robot state: hardware params, 4× motor states (`Moto_parameter`), servo, voltage, PI gains
- `Robot_Parament_InitTypeDef` — Wheel spacing, axle spacing, wheel circumference, gear ratio, encoder precision
- `Moto_parameter` — Per-motor: `Target` (m/s), `Encoder` (feedback), `Output` (PWM)

**`BALANCE/Inc/balance_task.h`:**
- `ROBOT_CONTROL_t` — Target velocities (Vx, Vy, Vz), active control mode, smoothing parameters
- `PI_CONTROLLER` — Incremental PI regulator: `Bias`, `LastBias`, `kp`, `ki`, `Output` (float, truncated to int on output)
- `AlphaBeta_Filter_t` — Alpha-Beta predictor-corrector filter: position estimate `x`, velocity estimate `v`, tuning params `alpha`/`beta`/`dt`
- `SEND_DATA` — 24-byte telemetry frame (header 0x7B, footer 0x7D, BCC checksum)
- `AKM_SERVO_UNLOCK_t` — Ackermann servo non-self-locking detection (top-tier only)
- `ROBOT_SELFCHECK_t` — Self-check error flags and encoder feedback verification

### PI Speed Controller

Incremental PI formula (in `balance_task.c`):
```
ΔOutput = Kp × (Bias − LastBias) + Ki × Bias
Output += ΔOutput          (float accumulation, cast to int on return)
```
Where `Bias = Target − Encoder_feedback`. `Output` is `float` internally to preserve small increments at low speed; truncated to `int` only when applied to PWM. Default gains: `VEL_KP=300, VEL_KI=300` (overridable per variant). One `PI_CONTROLLER` instance per motor (`PI_MotorA` through `PI_MotorD`) plus `PI_Servo` for AKM.

### Safety Features

- **CONTROL_DELAY** = 1000 ticks (10 seconds at 100Hz): robot ignores motion commands during startup IMU calibration and self-check
- **Self-check**: After CONTROL_DELAY, briefly drives at 0.2 m/s to verify encoder feedback
- **Command loss timeout**: `robot_control.command_lostcount` increments each Balance_task cycle; if no new command received, `UnResponseControl()` stops all motors
- **Parking mode**: Auto-clears PWM residuals when stationary to reduce power/noise

### Control Modes

Selected at runtime via bitfield enum: `_ROS_Control`, `_PS2_Control`, `_APP_Control`, `_RC_Control`, `_CAN_Control`, `_USART_Control`

### Communication Interfaces

- **UART1** (PA9/PA10, 115200) — Debug output
- **UART3** (PB10/PB11 or PD8/PD9, 115200) — ROS primary telemetry
- **UART4** (PC10/PC11 or PA0/PA1, 9600) — Bluetooth APP control
- **CAN1** — Control (ID 0x181: Vx/Vy/Vz) and telemetry; pin mapping varies by hardware version
- **USB Host** — Xbox 360, PS2 Classic, HID gamepad support

**Control frame format** (11 bytes): `[0x7B][cmd_type(2)][Vx(2)][Vy(2)][Vz(2)][BCC][0x7D]`

**Telemetry frame** (24 bytes, 20Hz): Velocities (×1000), IMU accel/gyro (ROS frame: IMU-Y→X, -IMU-X→Y, IMU-Z→Z), battery voltage, BCC checksum.

UART callback handlers: `HARDWARE/uartx_callback.c`. CAN callbacks: `HARDWARE/can_callback.c`. Timer callbacks: `HARDWARE/timx_callback.c`.

### Chassis Variant Selection

Each chassis type supports multiple hardware variants (e.g., Ackermann has 10). Selected at runtime by reading ADC potentiometer (`CarMode_Ch`) in `CarType/akm_robot_init.c` (and equivalent files). Each variant configures: wheel spacing, axle spacing, gear ratio, encoder resolution, wheel diameter.

### AKM (Ackermann) Specifics

Unlike other chassis types:
- **2 drive motors** (Motor_A left rear, Motor_B right rear) **+ 1 steering servo** (front axle)
- **Encoder**: T-method (period measurement via EXTI edge detection + TIM6 free-run counter) — not quadrature mode
- **Encoder C/D not initialized** (only 2 wheels are driven)
- **ADC2**: Initialized only for AKM — reads servo potentiometer feedback
- **Steering**: UART Vz (angular velocity) is converted to steering angle via `Akm_Vz_to_Angle()` before kinematics
- **Inverse kinematics**: `InverseKinematics_akm(Vx, Vz)` → differential wheel speeds + servo angle (polynomial-fitted PWM mapping)
- **Speed feedback pipeline**: T-method + M-method encoder fusion (`AKM_MixEncoderFeedback`) → Alpha-Beta filter with static-zone and reversal reset (`AKM_FilterWheelFeedback`). Filter instances `AB_MotorA`/`AB_MotorB` (alpha=0.18, beta=0.03, dt=0.01s) in `balance_task.c`
- **Servo non-self-locking**: `Servo_UnLock_Check` / `AKM_SERVO_UNLOCK_t` for top-tier models

Other chassis types (DIFF, MEC, 4WD, OMNI) use quadrature encoders via `Read_Encoder()` (TIM2/3/4/5) and have their own `InverseKinematics_*()` and `Kinematics_*()` functions.

### Conditional Compilation

Heavy use of `#if defined AKM_CAR` / `#elif defined DIFF_CAR` / etc. throughout `balance_task.c`, `data_task.c`, `show_task.c`, `motor.c`, `encoder.c`, and others. Only one chassis define is active per build target.

## Coding Conventions

- All source files use **UTF-8 with BOM** (`utf-8-sig`) encoding — converted from GB2312/GBK in March 2026 for Keil AC5 compatibility
- Uses **STM32 Standard Peripheral Library** (SPL) functions (`GPIO_Init`, `TIM_TimeBaseInit`, etc.) — not HAL or LL drivers
- **Include pattern**: Most `.c` files include only their own `.h`, which includes `system.h` (master include pulling in all drivers + FreeRTOS)
- **Header guards**: `#ifndef __NAME_H` / `#define __NAME_H` (double underscore prefix)
- **Naming**: Types/structs `PascalCase_t`, macros `UPPER_CASE`, functions `PascalCase` or mixed, variables mixed
- **Motor indices**: Motor_A (1), Motor_B (2), Motor_C (3), Motor_D (4)
- **Comment style**: Bilingual (English followed by Chinese). Function banners use `/**** ... ****/` with Function/Input/Output blocks. Inline comments use `//`
- **FreeRTOS**: Task functions `void TaskName(void *pvParameters)`, critical sections via `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()`
