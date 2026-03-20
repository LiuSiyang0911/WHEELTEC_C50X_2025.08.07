# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WHEELTEC C50X multi-chassis robot firmware. Runs on **STM32F407VE** (Cortex-M4) with **FreeRTOS**. Supports 5 robot chassis types (Ackermann, Differential, Mecanum, 4WD, Omnidirectional) with 48+ hardware variants selected at runtime via ADC potentiometer.

Language: C (bilingual comments in Chinese and English throughout).

## Build System

**IDE**: Keil µVision 5 (ARM Compiler v5.06)
**Project file**: `USER/WHEELTEC.uvprojx`
**Output**: `OBJ/` directory (hex, map, object files)

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

Build from Keil: select target, then Build (F7). No Makefile or CLI build available.

## Architecture

### Directory Layout

- **USER/** - Entry point (`main.c`), system headers (`system.h`), Keil project files
- **BALANCE/** - FreeRTOS task implementations (motion control, data output, display, LED, IMU)
- **BALANCE/Inc/** - Headers for task files and control structures
- **CarType/** - Per-chassis-type robot parameter initialization (wheel spacing, gear ratio, encoder config)
- **CarType/Inc/** - Robot config structures (`robot_init.h`)
- **HARDWARE/** - Peripheral drivers (motor, encoder, UART, CAN, I2C, ADC, OLED, USB host, etc.)
- **HARDWARE/Inc/** - Hardware driver headers
- **FWLIB/** - STM32F4xx Standard Peripheral Library (vendor code, do not modify)
- **FreeRTOS/** - FreeRTOS kernel (vendor code, do not modify)
- **CORE/** - ARM CMSIS headers (vendor code)
- **MiddleWares/** - USB Host middleware library (vendor code)
- **USB_HOST/** - USB host application layer (gamepad adapters)
- **SYSTEM/** - Low-level system init (clocks, delay, flash)

### FreeRTOS Tasks

All tasks created in `USER/main.c` → `start_task()`:

| Task | File | Priority | Rate | Purpose |
|------|------|----------|------|---------|
| Balance_task | `BALANCE/balance_task.c` | 4 | 100Hz | Core motion control, PI regulators, kinematics |
| data_task | `BALANCE/data_task.c` | 4 | 20Hz | Telemetry TX on UART1/CAN |
| MPU6050_task / ICM20948_task | `BALANCE/imu_task.c` | 3 | 100Hz | IMU sensor read + Kalman filter |
| show_task | `BALANCE/show_task.c` | - | - | OLED display |
| led_task | `BALANCE/led_task.c` | - | - | Status LEDs |
| pstwo_task | `BALANCE/ps2_task.c` | - | - | PS2 controller (v1.0 hardware only) |

### Hardware Version Detection

Two hardware revisions detected at runtime via `SysVal.HardWare_Ver`:
- **V1_0**: MPU6050 IMU, CAN on PA11/PA12, PS2 controller support
- **V1_1**: ICM20948 IMU, CAN on PD0/PD1, no PS2

### Key Data Structures (defined in `BALANCE/Inc/balance_task.h`)

- `ROBOT_CONTROL_t` - Target velocities (Vx, Vy, Vz) and active control mode
- `PI_CONTROLLER` - PI speed regulators for 4 motors + servo
- `SEND_DATA` - 24-byte telemetry frame (header 0x7B, footer 0x7D)
- `ROBOT_t` - Robot parameters and motor states

### Control Modes

Defined as enums, selected at runtime: `_ROS_Control`, `_PS2_Control`, `_APP_Control`, `_RC_Control`, `_CAN_Control`, `_USART_Control`

### Communication Interfaces

- **UART1** (PA9/PA10) - Data output: 24-byte telemetry frames
- **UART3** (PB10/PB11 or PD8/PD9) - App control input
- **UART4** (PC10/PC11 or PA0/PA1) - Bluetooth
- **CAN1** - Control and telemetry (pin mapping varies by hardware version)
- **USB Host** - Xbox 360, PS2 Classic, HID gamepad support

### Chassis Variant Selection

Each chassis type supports multiple hardware variants (e.g., Ackermann has 10). The variant is selected at runtime by reading an ADC potentiometer (`CarMode_Ch`) in `CarType/akm_robot_init.c` (and equivalent files for other chassis types). Each variant configures: wheel spacing, axle spacing, gear ratio, encoder resolution, wheel diameter.

## Coding Conventions

- All source files use **UTF-8 with BOM** (`utf-8-sig`) encoding — converted from GB2312/GBK in March 2026 for Keil AC5 compatibility
- Most functions and variables use English names with Chinese comments alongside
- Motor indices: Motor_A (1), Motor_B (2), Motor_C (3), Motor_D (4)
- Rate macros defined in `USER/system.h`: `RATE_1HZ` through `RATE_1000HZ`
- UART callback handlers are in `HARDWARE/uartx_callback.c` and `HARDWARE/can_callback.c`
- Timer callbacks in `HARDWARE/timx_callback.c`
