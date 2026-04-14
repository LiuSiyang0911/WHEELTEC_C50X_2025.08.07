# AKM Fixed PWM AFC Test Design

**Date:** 2026-04-14

**Goal**

在阿克曼 `USART1` 固定 PWM 调试链路上，增加一个仅用于直行前进场景的 AFC 测试路径，使车辆能够以固定 PWM 基值前进，同时叠加 AFC 补偿量，且不使用电机 PI 速度闭环参与补偿。

## Current Context

- 当前分支已经具备 AKM AFC 实现，核心函数为 `AKM_AFC_Compensate()`，其输入假定为“基础 PWM + 目标/反馈速度”。
- 当前 `USART1` 已支持两种目标模式：
  - `UART_TARGET_MODE_SPEED`：A/B 轮目标速度
  - `UART_TARGET_MODE_PWM`：A/B 轮固定 PWM
- `UART_TARGET_MODE_PWM` 在 `ResponseControl()` 中会直接进入旁路输出，并在进入该分支时执行 `AKM_AFC_ResetAll()`，因此 AFC 无法在固定 PWM 场景下学习或补偿。
- AKM 的方向映射由 `Apply_AKM_UartTargetFromApp()` 完成；其中严格的直行前进场景为 `appkey.DirectionFlag == 1`。

## Scope

本次只修改以下行为：

- 仅在 `AKM_CAR`
- 仅在 `robot_control.uart_target_mode == UART_TARGET_MODE_PWM`
- 仅在 `robot_control.uart_target_valid == 1`
- 仅在 `appkey.DirectionFlag == 1`

其余方向与模式保持现状，不扩展协议，不新增调试命令，不改上位机数据格式。

## Behavior Design

### 1. PWM 前进测试模式

当 `USART1` 下发 `SET_PWM_AB` 后，如果 APP 方向键处于直行前进（`DirectionFlag == 1`），控制链路应执行：

1. 仍复用现有 `Apply_AKM_UartTargetFromApp()` 方向映射结果，得到 A/B 轮基础 PWM。
2. 不进入 `Incremental_MOTOR()`，即不使用 PI 速度闭环。
3. 将基础 PWM 作为 `AKM_AFC_Compensate()` 的 `base_pwm` 输入。
4. 将基础 PWM 的等效目标量作为 AFC 的参考目标输入，使 AFC 能够根据实际反馈速度生成增量 PWM。
5. 最终输出为 `base_pwm + afc_output`，再按现有 AKM 电机正负方向规则调用 `Set_Pwm()`。

### 2. 非前进场景保持原行为

当 `UART_TARGET_MODE_PWM` 有效但不满足 `DirectionFlag == 1` 时：

- 保持当前纯固定 PWM 输出行为；
- 不尝试在转向、倒车、原地转向或停止场景学习 AFC；
- 进入这些场景时复位 AFC 状态，避免把非测试动作误学习进补偿器。

### 3. 状态切换规则

以下情况需要复位 AFC：

- 退出 `DirectionFlag == 1` 的前进 PWM 测试场景；
- PWM 目标为 0；
- 小车进入失能或无响应停机；
- 现有 AFC 内部判定条件要求复位时。

这样可以保证 AFC 只在单一、稳定的测试场景中学习。

## Code Design

### Files

- Modify: `BALANCE/balance_task.c`
- Modify: `BALANCE/Inc/balance_task.h`

### Planned Code Changes

#### `BALANCE/balance_task.c`

- 为 AKM 的 UART PWM 前进测试增加一个独立的辅助路径，职责是：
  - 判断当前是否为“PWM + 前进”测试条件；
  - 在该条件下执行“基础 PWM + AFC”输出；
  - 在非测试条件下复位 AFC 并回退到当前纯 PWM 输出。
- `ResponseControl()` 不再对所有 PWM 模式一律 `AKM_AFC_ResetAll()` 后直接输出。
- 继续复用 `Apply_AKM_UartTargetFromApp()` 生成的 `robot.MOTOR_A.Target` / `robot.MOTOR_B.Target` / `robot.MOTOR_A.Output` / `robot.MOTOR_B.Output`，避免复制方向映射逻辑。

#### `BALANCE/Inc/balance_task.h`

- 为新增的 AKM PWM AFC 辅助函数补充静态声明。

## Data Flow

`USART1 SET_PWM_AB`
-> `Set_UartTargetPwm()`
-> `Apply_AKM_UartTargetFromApp()`
-> `ResponseControl()`

分两种情况：

- 前进测试场景：
  - 基础 PWM 写入 `robot.MOTOR_A/B.Output`
  - 当前反馈速度来自 `robot.MOTOR_A/B.Encoder`
  - 调用 `AKM_AFC_Compensate()`
  - 输出 `基础 PWM + AFC`

- 非前进场景：
  - 直接使用基础 PWM 输出
  - AFC 复位

## Error Handling And Safety

- 不修改 `Set_UartTargetPwm()` 的限幅逻辑，仍要求 PWM 输入在 `0..FULL_DUTYCYCLE`。
- AFC 测试场景中，如果基础 PWM 为 0，则直接复位 AFC 并输出 0。
- 不改变 `UnResponseControl()`、`UartTarget_ClearMotionState()` 中已有的停机和清零逻辑，只保证新增路径与这些逻辑兼容。
- 不修改舵机控制链路；前进 PWM AFC 测试只影响 A/B 驱动轮。

## Verification Design

### Build Verification

使用 Keil 批量编译 `Akm_Car` 目标，要求日志末尾出现：

- `"..\\OBJ\\Akm_Car.axf" - 0 Error(s), 0 Warning(s).`
- `Build Time Elapsed: ...`

### Runtime Verification

实机验证重点：

1. 下发 `SET_PWM_AB` 后，`DirectionFlag == 1` 时车辆以固定 PWM 前进。
2. 调试帧中的 `afc_output_A/B` 不再恒为 0，而是随扰动建立补偿量。
3. 切换到非前进方向后，AFC 输出应回零或重新学习，不保留错误历史。
4. 其他方向的固定 PWM 行为保持原样。

## Non-Goals

- 不修改 AFC 算法参数。
- 不新增 PID/PI 混合模式。
- 不新增上位机命令、Flash 参数或 UI 配置。
- 不处理非 AKM 车型。
