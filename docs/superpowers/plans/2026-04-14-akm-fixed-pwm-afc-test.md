# AKM Fixed PWM AFC Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make AKM `USART1` fixed-PWM control support AFC compensation only in straight-forward motion (`DirectionFlag == 1`) without using the motor PI loop.

**Architecture:** Keep the existing UART PWM command and APP direction mapping intact, then add a narrow AKM-only helper path inside `balance_task.c` that converts the already-mapped PWM outputs into “base PWM + AFC” when the car is in the forward test scenario. All non-forward PWM cases continue using the current open-loop output path and explicitly reset AFC state to avoid cross-scene learning.

**Tech Stack:** STM32F4 C firmware, FreeRTOS task loop, Keil µVision 5 / ARMCC5, existing AKM AFC logic in `BALANCE/balance_task.c`

---

### Task 1: Add AKM UART PWM AFC helper boundary

**Files:**
- Modify: `BALANCE/balance_task.c`
- Modify: `BALANCE/Inc/balance_task.h`

- [ ] **Step 1: Add the helper declarations before implementation**

```c
#if defined AKM_CAR
static uint8_t AKM_IsUartPwmForwardAfcTest(void);
static void Apply_AKM_UartPwmForwardAfc(void);
#endif
```

- [ ] **Step 2: Implement the forward-scene detector**

```c
static uint8_t AKM_IsUartPwmForwardAfcTest(void)
{
	if( robot_control.uart_target_mode != UART_TARGET_MODE_PWM ) return 0;
	if( robot_control.uart_target_valid == 0 ) return 0;
	if( appkey.DirectionFlag != 1 ) return 0;
	if( robot_control.uart_pwm_a == 0 && robot_control.uart_pwm_b == 0 ) return 0;
	return 1;
}
```

- [ ] **Step 3: Implement the fixed-PWM plus AFC output path**

```c
static void Apply_AKM_UartPwmForwardAfc(void)
{
	if( AKM_IsUartPwmForwardAfcTest() == 0 )
	{
		AKM_AFC_ResetAll();
		Apply_AKM_UartPwmOutput(robot.MOTOR_A.Output,robot.MOTOR_B.Output);
		return;
	}

	robot.MOTOR_A.Output = AKM_AFC_Compensate(&afc_motor_a,robot.MOTOR_A.Output,robot.MOTOR_A.Target,robot.MOTOR_A.Encoder,robot.MOTOR_A.Encoder);
	robot.MOTOR_B.Output = AKM_AFC_Compensate(&afc_motor_b,robot.MOTOR_B.Output,robot.MOTOR_B.Target,robot.MOTOR_B.Encoder,robot.MOTOR_B.Encoder);
	Apply_AKM_UartPwmOutput(robot.MOTOR_A.Output,robot.MOTOR_B.Output);
}
```

- [ ] **Step 4: Inspect the edited declarations and helper block**

Run: `rg -n "AKM_IsUartPwmForwardAfcTest|Apply_AKM_UartPwmForwardAfc" BALANCE\\balance_task.c BALANCE\\Inc\\balance_task.h`
Expected: both symbols appear in declaration and implementation positions.

### Task 2: Route `ResponseControl()` through the new helper

**Files:**
- Modify: `BALANCE/balance_task.c`

- [ ] **Step 1: Replace the unconditional PWM bypass**

```c
if( robot_control.uart_target_mode == UART_TARGET_MODE_PWM && robot_control.uart_target_valid )
{
	Apply_AKM_UartPwmForwardAfc();
	return;
}
```

- [ ] **Step 2: Preserve AFC reset semantics for non-test scenes**

```c
if( AKM_IsUartPwmForwardAfcTest() == 0 )
{
	AKM_AFC_ResetAll();
}
```

Put this inside the helper so the PWM forward test is the only scene that keeps AFC learning state.

- [ ] **Step 3: Re-read the surrounding AKM control flow**

Run: `Get-Content BALANCE\\balance_task.c | Select-Object -Skip 1148 -First 160`
Expected: the PWM mode branch calls the helper, PI speed control remains unchanged for non-PWM paths, and `Set_Pwm()` direction handling is untouched.

### Task 3: Verify source consistency and compile `Akm_Car`

**Files:**
- Modify: `BALANCE/balance_task.c`
- Modify: `BALANCE/Inc/balance_task.h`

- [ ] **Step 1: Check the git diff for scope**

Run: `git diff -- BALANCE/Inc/balance_task.h BALANCE/balance_task.c`
Expected: only the new AKM helper declarations and the PWM routing change are present.

- [ ] **Step 2: Build the Keil target**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b D:\radar\car\WHEELTEC_C50X_2025.08.07\USER\WHEELTEC.uvprojx -j0 -t Akm_Car -o D:\radar\car\WHEELTEC_C50X_2025.08.07\USER\.vscode\uv4.log
```

Expected:

```text
"..\OBJ\Akm_Car.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: ...
```

- [ ] **Step 3: Read the end of the build log**

Run: `Get-Content USER\\.vscode\\uv4.log -Tail 20`
Expected: the final lines include zero errors, zero warnings, and elapsed time.

- [ ] **Step 4: Commit the implementation**

```bash
git add BALANCE/Inc/balance_task.h BALANCE/balance_task.c
git commit -m "feat: support AKM fixed PWM AFC forward test"
```
