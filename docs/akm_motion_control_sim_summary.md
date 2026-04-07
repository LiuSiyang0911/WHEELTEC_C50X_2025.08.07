# Type 4 顶配 AKM 小车运动控制仿真摘要

本文档只保留 AKM 顶配独立悬挂车型 `type 4` 的运动控制核心逻辑，适合作为仿真建模输入。

适用目标：

- `Akm_Car`
- `robot.type == 4`

核心来源代码：

- `USER/main.c`
- `USER/system.h`
- `USER/system.c`
- `BALANCE/balance_task.c`
- `BALANCE/data_task.c`
- `CarType/akm_robot_init.c`
- `CarType/Inc/robot_init.h`
- `HARDWARE/encoder.c`
- `HARDWARE/adc.c`
- `HARDWARE/Inc/motor.h`

## 1. 控制链路

`type 4` 的主闭环在 `Balance_task()` 中，以 100 Hz 执行。

可抽象为：

```text
输入命令(Vx, yaw_rate)
    -> Akm_Vz_to_Angle() 转成左前轮转角 delta_left
    -> Drive_Motor() 做限幅和平滑
    -> InverseKinematics_akm() 计算左右后轮目标速度和滑轨目标
    -> ResponseControl() 计算左右轮 PWM 和舵机 PWM
    -> 电机/舵机执行机构
    -> Get_Robot_FeedBack() 获取轮速和滑轨反馈
    -> 下一个 10 ms 控制周期
```

仿真必须保留的核心函数：

- `Akm_Vz_to_Angle()`
- `Drive_Motor()`
- `InverseKinematics_akm()`
- `ResponseControl()`
- `Get_Robot_FeedBack()`
- `Kinematics_akm_diff()`

## 2. 频率与时序

### 2.1 任务频率

- 控制主任务：`BALANCE_TASK_RATE = 100 Hz`
- 遥测任务：`DATA_TASK_RATE = 20 Hz`
- IMU 任务：`IMU_TASK_RATE = 100 Hz`

其中真正决定运动控制离散步长的是：

- 控制周期 `dt = 0.01 s`

### 2.2 上电延时与自检

`CONTROL_DELAY = 1000`，按 100 Hz 计算即 10 秒。

控制时序：

- 上电后前 10 秒，不进入正常运动控制
- `10.0 s ~ 12.0 s`：执行 `Drive_Motor(0.2f, 0, 0)` 做低速前进自检
- `12.0 s ~ 12.3 s`：停车
- `12.3 s` 之后：进入正常控制

### 2.3 丢指令保护

正常控制阶段中，`command_lostcount` 每个控制周期加一；超过 `BALANCE_TASK_RATE` 就清零目标速度。

等价于：

- 指令超时阈值 `1.0 s`

## 3. Type 4 固定参数

### 3.1 几何参数

来自 `CarType/akm_robot_init.c` 和 `CarType/Inc/robot_init.h`：

- `WheelSpacing = 0.595 m`
- `AxleSpacing = 0.525 m`
- `WheelDiameter = 0.254 m`
- `Wheel_Circ = pi * 0.254 = 0.7979645 m`
- `MIN_turn_radius = 1.200 m`

### 3.2 电机与编码器参数

- `GearRatio = 18`
- `EncoderAccuracy = 500`
- `EncoderMultiples = 4`
- `Encoder_precision = 4 * 500 * 18 = 36000`

### 3.3 速度环默认参数

`type 4` 在 `Robot_Select()` 中单独指定：

- `kp = 400`
- `ki = 100`
- `kd = 0`

### 3.4 其他控制参数

来自 `ROBOT_CONTROL_t_Init()`：

- `limt_max_speed = 3.5 m/s`
- `rc_speed = 500 mm/s`
- `smooth_MotorStep = 0.02`
- `smooth_ServoStep = 20`
- `LineDiffParam = 50`

### 3.5 舵机参数

来自 `Akm_ServoParam_Init()`：

- `Akm_Servo.Min = 1000`
- `Akm_Servo.Mid = 1500`
- `Akm_Servo.Max = 2000`

## 4. 控制输入定义

上位机常见输入是：

- `Vx`：前进速度，单位 `m/s`
- `Vz`：车体角速度，单位 `rad/s`

但 `type 4` 的 AKM 逆运动学实际接收的是：

- `Vx`
- `delta_left`：左前轮转角，单位 `rad`

所以仿真如果对齐上位机接口，必须先经过：

```c
TurnR = Vx / Vz;
delta_left = atan(AxleSpacing / (TurnR - WheelSpacing / 2));
```

对应固件函数：`Akm_Vz_to_Angle()`

边界处理：

- `Vx == 0` 或 `Vz == 0` 时，直接返回 `delta_left = 0`
- 固件会强制保证 `|TurnR| > WheelSpacing / 2`

## 5. 命令整形

`Drive_Motor()` 中先做输入限幅：

```c
T_Vx = clamp(T_Vx, -limt_max_speed, limt_max_speed);
T_Vy = clamp(T_Vy, -limt_max_speed, limt_max_speed);
T_Vz = clamp(T_Vz, -limt_max_speed, limt_max_speed);
```

对 `type 4` 实际有意义的是 `T_Vx` 和 `T_Vz`。

然后做平滑：

```c
smooth_Vx = Vel_SmoothControl(smooth_Vx, T_Vx, smooth_MotorStep);
smooth_Vy = Vel_SmoothControl(smooth_Vy, T_Vy, smooth_MotorStep);
smooth_Vz = T_Vz;
```

也就是说：

- 线速度做斜率限制
- 转向角输入不做平滑

`Vel_SmoothControl()` 的本质是固定步长逼近：

```c
if(now > target) now -= step;
else             now += step;
```

默认 `smooth_MotorStep = 0.02`，在 10 ms 控制周期下等价于：

- 最大线速度变化率约 `2.0 m/s^2`

## 6. Type 4 逆运动学

`type 4` 走的是 `InverseKinematics_akm()` 中独立悬挂分支。

### 6.1 转弯半径与左右后轮目标速度

`type 4` 使用的结构参数：

- 转向角限制约 `+/-25 deg`
- 连杆参数 `K = 0.441`

核心公式：

```c
TurnR = AxleSpacing / tan(delta_left) + K / 2;

v_left  = Vx * (TurnR - WheelSpacing / 2) / TurnR;
v_right = Vx * (TurnR + WheelSpacing / 2) / TurnR;
```

直行时：

```c
v_left  = Vx;
v_right = Vx;
```

### 6.2 滑轨目标值

`type 4` 不是直接输出舵机 PWM，而是先把前轮转角变成滑轨目标值：

```c
theta = delta_left - 1.170f;
pos = 95.15f * cos(theta)
    + sqrt(24025.0f - pow(95.15f * sin(theta) + 56.5f, 2))
    - 191.54f;
pos /= 0.0244f;
pos = clamp(pos, -2000, 2000);
```

对应函数：`SteeringStructure_Reverse()`

这一步输出的是：

- `robot.SERVO.Target`，即滑轨目标值，不是最终 PWM

## 7. 执行机构控制

### 7.1 左右轮增量式 PI

`Incremental_MOTOR()` 的公式：

```c
Bias = Target - Feedback;
DeltaOutput = kp * (Bias - LastBias)
            + ki * Bias
            + kd * (Bias - 2 * LastBias + LastestBias);
Output = Output + DeltaOutput;
Output = clamp(Output, -16800, 16800);
```

`type 4` 使用：

- `kp = 400`
- `ki = 100`
- `kd = 0`

零速保护：

- 当轮目标绝对值小于 `0.01 m/s` 时，PI 直接复位，PWM 输出置 0

### 7.2 AKM AFC 自适应前馈补偿

`type 4` 的轮速 PI 后面还有一层 `AKM_AFC_Compensate()`：

- 用轮速相位学习周期性误差
- 结果叠加到基础 PWM 上

关键参数：

- `AKM_AFC_DT = 0.01`
- `AKM_AFC_MIN_TARGET_SPEED = 0.10 m/s`
- `AKM_AFC_MIN_FEEDBACK_SPEED = 0.08 m/s`
- `AKM_AFC_RESET_SPEED = 0.03 m/s`
- `AKM_AFC_LMS_MU = 20.0`
- `AKM_AFC_LEAK = 0.999`
- `AKM_AFC_MAX_PWM = 2000`

仿真建议：

- 第一版可以先忽略 AFC
- 如果想更接近实车，再把 AFC 补进去

### 7.3 Type 4 舵机 PI

`type 4` 的舵机是闭环控制，输入为滑轨目标值，反馈为滑轨 ADC。

舵机 PI 摘要：

```c
servo_dir = -1;

if(target > 0 || current < 200) {
    servo_kp = 0.002 * 1.3;
    servo_ki = 0.006 * 1.3;
} else {
    servo_kp = 0.002;
    servo_ki = 0.006;
}

Output += (servo_kp * servo_dir) * (Bias - LastBias)
       +  (servo_ki * servo_dir) * Bias;
Output = clamp(Output, 1000, 2000);
```

其中 `servo_dir = -1` 是因为 `type 4` 舵机反装。

### 7.4 舵机低速模式与平滑

低速模式触发时：

- `servo_kp = 0.001`
- `servo_ki = 0.003`
- 持续时间约 `2 s`

然后再做 PWM 平滑：

```c
if(smooth_Servo > Output) smooth_Servo -= smooth_ServoStep;
if(smooth_Servo < Output) smooth_Servo += smooth_ServoStep;
```

默认：

- `smooth_ServoStep = 20`

## 8. 反馈测量

### 8.1 左右轮速度反馈

`type 4` 的轮速反馈融合了两种测速源。

M 法测速：

```c
m_speed =
    encoder_count_in_10ms
    * 100
    / Encoder_precision
    * Wheel_Circ
    / WheelDiff;
```

T 法测速：

```c
t_speed =
    sign
    * (4 * 1e6 / total_dt_us)
    * (Wheel_Circ / (Encoder_precision / 4));
```

T 法关键参数：

- `T_METHOD_WINDOW_PULSES = 4`
- `T_METHOD_MIN_DT_US = 20`
- `T_METHOD_MAX_DT_US = 500000`
- 超过 `100000 us` 没有新脉冲则清零该通道

### 8.2 T 法 / M 法融合

`AKM_MixEncoderFeedback()` 的逻辑：

- 低速优先 T 法
- 高速优先 M 法
- 中间速度线性混合

阈值：

- `AKM_BLEND_T_SPEED_LOW = 0.12 m/s`
- `AKM_BLEND_M_SPEED_HIGH = 0.28 m/s`

### 8.3 Alpha-Beta 滤波

融合后的轮速再经过 Alpha-Beta 滤波：

- `alpha = 0.18`
- `beta = 0.03`
- `dt = 0.01 s`

并带两条保护逻辑：

- 明显静止时直接复位
- 明显换向时直接复位

### 8.4 滑轨反馈

`type 4` 的转向反馈是滑轨 ADC：

```c
robot.SERVO.Encoder = get_DMA_SlideRes();
robot.SERVO.Encoder = Slide_Mean_Filter(robot.SERVO.Encoder);
```

其中平均滤波窗口长度：

- `FILTERING_TIMES = 20`

## 9. 正运动学输出

固件在 `data_task.c` 中仍使用简单的左右轮模型：

```c
vx = (v_left + v_right) / 2;
vy = 0;
wz = (v_right - v_left) / WheelSpacing;
```

注意：

- 这是固件对外发送速度时使用的近似模型
- 如果做更像真实 AKM 的仿真，车体姿态积分建议改用自行车模型或阿克曼模型

## 10. 推荐的最小仿真状态量

建议至少保留这些状态：

```text
车体:
    x, y, yaw

驱动轮:
    v_left, v_right

转向:
    delta_left
    servo_slide_pos
    servo_pwm

控制器内部:
    PI_MotorA.Output, PI_MotorA.LastBias
    PI_MotorB.Output, PI_MotorB.LastBias
    PI_Servo.Output,  PI_Servo.LastBias
    smooth_Vx
    smooth_Vz
    smooth_Servo

可选增强:
    AFC 相位 / 增益
    Alpha-Beta 滤波内部状态
```

## 11. 推荐的仿真分层

### 11.1 第一版

先实现：

1. `Vx + yaw_rate -> delta_left`
2. `delta_left -> 左右轮目标速度 + 滑轨目标`
3. 左右轮增量式 PI
4. 舵机 PI + 平滑
5. 一阶电机对象 + 一阶舵机对象
6. 自行车模型或阿克曼模型积分位姿

先忽略：

- AFC
- 自动回充
- 舵机非自锁
- 安全检测细节

### 11.2 第二版

再补：

1. T 法 / M 法融合测速
2. Alpha-Beta 轮速滤波
3. 10 秒启动延时
4. 1 秒丢指令超时

## 12. 一段最小离散仿真骨架

```c
// dt = 0.01 s
void sim_step(float cmd_vx, float cmd_wz)
{
    float delta_left;
    float left_target;
    float right_target;
    float servo_slide_target;

    delta_left = Akm_Vz_to_Angle(cmd_vx, cmd_wz);

    smooth_vx = slew(smooth_vx, clamp(cmd_vx, -3.5f, 3.5f), 0.02f);
    smooth_delta = delta_left;

    inverse_akm_type4(
        smooth_vx,
        smooth_delta,
        &left_target,
        &right_target,
        &servo_slide_target
    );

    pwm_left  = incremental_pi(&pi_left,  left_feedback,  left_target);
    pwm_right = incremental_pi(&pi_right, right_feedback, right_target);

    servo_pwm = servo_pi_type4(&pi_servo, servo_feedback, servo_slide_target);

    left_feedback  = motor_plant_left(pwm_left);
    right_feedback = motor_plant_right(pwm_right);
    servo_feedback = servo_plant(servo_pwm);

    vx = 0.5f * (left_feedback + right_feedback);
    wz = (right_feedback - left_feedback) / 0.595f;
}
```

如果你下一步要做 Python/Matlab/Simulink 仿真，按这个文档直接展开就够了。
