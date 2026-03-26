#include "debug_uart.h"
#include "balance_task.h"
#include "robot_init.h"
#include "encoder.h"
#include <string.h>

#if defined AKM_CAR
/* DOB disturbance estimates defined in balance_task.c */
/* DOB扰动估计变量，定义于balance_task.c               */
extern float disturbance_a;
extern float disturbance_b;
#endif

/*===========================================================================
 * 调试串口模块 - UART1 DMA发送 + 中断接收
 *
 * TX: DMA2_Stream7_Channel4 → USART1_TX, 100Hz数据帧(40B) + 按需参数帧(40B)
 * RX: USART1 RXNE中断 → 状态机解析命令帧
 *===========================================================================*/

/* ---- TX缓冲区与DMA状态 ---- */
static uint8_t debug_tx_buf[64];       /* 发送缓冲区(最长帧40字节,预留安全空间) */
static volatile uint8_t dma_busy = 0;  /* DMA传输中标志 */
static volatile uint8_t param_reply_pending = 0; /* 参数回复请求标志 */

/* ---- RX状态机 ---- */
typedef enum {
	RX_WAIT_HEADER1 = 0,
	RX_WAIT_HEADER2,
	RX_WAIT_CMD,
	RX_WAIT_LEN,
	RX_WAIT_PAYLOAD,
	RX_WAIT_CHECKSUM
} DebugRxState_t;

static DebugRxState_t rx_state = RX_WAIT_HEADER1;
static uint8_t rx_cmd = 0;
static uint8_t rx_len = 0;
static uint8_t rx_idx = 0;
static uint8_t rx_buf[DEBUG_RX_BUF_LEN];
static uint8_t rx_checksum = 0;
static volatile uint8_t rx_idle_count = 0;
#define DEBUG_RX_IDLE_THRESHOLD_CYCLES 5  /* 5个Balance_task周期(当前100Hz≈50ms) */

/* ---- 内部函数声明 ---- */
static void pack_float(uint8_t *buf, float val);
static void pack_int16(uint8_t *buf, int16_t val);
static float unpack_float(const uint8_t *buf);
static int16_t unpack_int16(const uint8_t *buf);
static void debug_execute_cmd(uint8_t cmd, const uint8_t *payload, uint8_t len);
static void dma_start_tx(uint16_t len);

/*===========================================================================
 * memcpy辅助函数 - 保证ARM对齐安全
 *===========================================================================*/
static void pack_float(uint8_t *buf, float val)
{
	memcpy(buf, &val, 4);
}

static void pack_int16(uint8_t *buf, int16_t val)
{
	memcpy(buf, &val, 2);
}

static float unpack_float(const uint8_t *buf)
{
	float val;
	memcpy(&val, buf, 4);
	return val;
}

static int16_t unpack_int16(const uint8_t *buf)
{
	int16_t val;
	memcpy(&val, buf, 2);
	return val;
}

static int is_valid_pid(float val)
{
	/* 1. NaN检查 */
	if (val != val) return 0;
	/* 2. 范围检查 (同时拦截 ±Inf) */
	if (val < 0.0f || val > 50000.0f) return 0;
	return 1;
}

static int is_valid_speed(float val)
{
	if (val != val) return 0;
	if (val < 0.0f || val > 10.0f) return 0;
	return 1;
}

/*===========================================================================
 * DMA2_Stream7 初始化 - USART1_TX
 *===========================================================================*/
void Debug_UART_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 使能DMA2时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* 复位Stream7 */
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);

	/* 配置DMA */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;              /* USART1_TX = Ch4 */
	DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)debug_tx_buf;
	DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize          = DEBUG_DATA_FRAME_LEN;
	DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;           /* 单次模式 */
	DMA_InitStructure.DMA_Priority            = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	/* 使能DMA传输完成中断 */
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

	/* 配置NVIC */
	NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 使能USART1的DMA发送请求 */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

/*===========================================================================
 * 启动一次DMA发送
 *===========================================================================*/
static void dma_start_tx(uint16_t len)
{
	DMA_Cmd(DMA2_Stream7, DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);

	DMA_SetCurrDataCounter(DMA2_Stream7, len);

	/* 清除所有中断标志 */
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 |
	              DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);

	dma_busy = 1;
	DMA_Cmd(DMA2_Stream7, ENABLE);
}

/*===========================================================================
 * 等待DMA发送完成 (带超时保护)
 * 用于在同步发送(any_printf)前确保DMA传输已结束,避免USART1数据冲突
 *===========================================================================*/
void Debug_WaitTxDone(void)
{
	/* 1. 等待DMA传输完成 */
	volatile uint32_t timeout = 100000;  /* ~10ms@168MHz */
	while (dma_busy && --timeout > 0);

	if (dma_busy)
	{
		/* DMA异常未完成, 强制清理 */
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 |
		              DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
		dma_busy = 0;
	}

	/* 2. 等待USART移位寄存器发空(TC=Transmission Complete, 不仅仅是DR空)
	 *    即使上面DMA超时被强制关闭, 仍需等TC确保线上最后一个字节发完 */
	timeout = 100000;
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET && --timeout > 0);
}

/*===========================================================================
 * DMA2_Stream7 传输完成中断
 *===========================================================================*/
void DMA2_Stream7_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
	{
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
		DMA_Cmd(DMA2_Stream7, DISABLE);
		dma_busy = 0;
	}
}

/*===========================================================================
 * TX数据帧组装与发送 (从Balance_task 100Hz调用)
 *
 * 帧格式 (48字节):
 *   [0-1]   0xAA 0x55
 *   [2]     FrameID = 0x01
 *   [3-6]   t_raw_A       (float, T法原始速度 m/s)
 *   [7-10]  t_raw_B       (float, T法原始速度 m/s)
 *   [11-14] m_raw_A       (float, M法原始速度 m/s)
 *   [15-18] m_raw_B       (float, M法原始速度 m/s)
 *   [19-22] final_A       (float, 融合后反馈速度 m/s)
 *   [23-26] final_B       (float, 融合后反馈速度 m/s)
 *   [27-30] target_A      (float, 目标速度 m/s)
 *   [31-34] target_B      (float, 目标速度 m/s)
 *   [35-36] output_A      (int16, PI原始输出 PWM)
 *   [37-38] output_B      (int16, PI原始输出 PWM)
 *   [39-42] disturbance_A (float, DOB扰动估计 AKM only, else 0.0f)
 *   [43-46] disturbance_B (float, DOB扰动估计 AKM only, else 0.0f)
 *   [47]    checksum      (XOR of bytes 0..46)
 *===========================================================================*/
void Debug_SendDataFrame(void)
{
	uint8_t i;
	uint8_t xor_sum;

	/* DMA忙则跳过本帧 */
	if (dma_busy) return;

	/* 如果有参数回复请求,优先发送参数帧 */
	if (param_reply_pending)
	{
		param_reply_pending = 0;
		Debug_SendParamFrame();
		return;
	}

	/* 组装帧头 */
	debug_tx_buf[0] = DEBUG_FRAME_HEADER1;
	debug_tx_buf[1] = DEBUG_FRAME_HEADER2;
	debug_tx_buf[2] = DEBUG_FRAME_DATA;

	/* T法编码器原始速度 (Kalman滤波前) */
	#if defined AKM_CAR
		pack_float(&debug_tx_buf[3],  encoder_T_velocity_raw[0]);
		pack_float(&debug_tx_buf[7],  encoder_T_velocity_raw[1]);
		pack_float(&debug_tx_buf[11], akm_encoder_m_raw[0]);
		pack_float(&debug_tx_buf[15], akm_encoder_m_raw[1]);
	#else
		pack_float(&debug_tx_buf[3],  0.0f);
		pack_float(&debug_tx_buf[7],  0.0f);
		pack_float(&debug_tx_buf[11], 0.0f);
		pack_float(&debug_tx_buf[15], 0.0f);
	#endif

	/* 融合并滤波后的闭环反馈速度 */
	pack_float(&debug_tx_buf[19], robot.MOTOR_A.Encoder);
	pack_float(&debug_tx_buf[23], robot.MOTOR_B.Encoder);

	/* 目标速度 */
	pack_float(&debug_tx_buf[27], robot.MOTOR_A.Target);
	pack_float(&debug_tx_buf[31], robot.MOTOR_B.Target);

	/* PI原始输出PWM */
	pack_int16(&debug_tx_buf[35], (int16_t)robot.MOTOR_A.Output);
	pack_int16(&debug_tx_buf[37], (int16_t)robot.MOTOR_B.Output);

	/* DOB扰动估计值 / DOB disturbance estimates */
	#if defined AKM_CAR
		pack_float(&debug_tx_buf[39], disturbance_a);
		pack_float(&debug_tx_buf[43], disturbance_b);
	#else
		pack_float(&debug_tx_buf[39], 0.0f);
		pack_float(&debug_tx_buf[43], 0.0f);
	#endif

	/* XOR校验 */
	xor_sum = 0;
	for (i = 0; i < 47; i++)
		xor_sum ^= debug_tx_buf[i];
	debug_tx_buf[47] = xor_sum;

	/* 触发DMA发送 */
	dma_start_tx(DEBUG_DATA_FRAME_LEN);
}

/*===========================================================================
 * TX参数回复帧 (40字节)
 *
 *   [0-1]   0xAA 0x55
 *   [2]     FrameID = 0x02
 *   [3-6]   A_kp     (float)
 *   [7-10]  A_ki     (float)
 *   [11-14] A_kd     (float)
 *   [15-18] B_kp     (float)
 *   [19-22] B_ki     (float)
 *   [23-26] B_kd     (float)
 *   [27-30] rc_speed        (float)
 *   [31-34] limt_max_speed  (float)
 *   [35-38] smooth_MotorStep(float)
 *   [39]    checksum (XOR of bytes 0..38)
 *===========================================================================*/
void Debug_SendParamFrame(void)
{
	uint8_t i;
	uint8_t xor_sum;

	/* DMA忙则跳过 */
	if (dma_busy) return;

	debug_tx_buf[0] = DEBUG_FRAME_HEADER1;
	debug_tx_buf[1] = DEBUG_FRAME_HEADER2;
	debug_tx_buf[2] = DEBUG_FRAME_PARAM;

	pack_float(&debug_tx_buf[3],  PI_MotorA.kp);
	pack_float(&debug_tx_buf[7],  PI_MotorA.ki);
	pack_float(&debug_tx_buf[11], PI_MotorA.kd);
	pack_float(&debug_tx_buf[15], PI_MotorB.kp);
	pack_float(&debug_tx_buf[19], PI_MotorB.ki);
	pack_float(&debug_tx_buf[23], PI_MotorB.kd);
	pack_float(&debug_tx_buf[27], robot_control.rc_speed);
	pack_float(&debug_tx_buf[31], robot_control.limt_max_speed);
	pack_float(&debug_tx_buf[35], robot_control.smooth_MotorStep);

	xor_sum = 0;
	for (i = 0; i < 39; i++)
		xor_sum ^= debug_tx_buf[i];
	debug_tx_buf[39] = xor_sum;

	dma_start_tx(DEBUG_PARAM_FRAME_LEN);
}

/*===========================================================================
 * RX状态机 - 逐字节解析命令帧
 *
 * 帧格式: 0xAA 0x55 | CmdID(1B) | Length(1B) | Payload(N B) | Checksum(1B)
 * Checksum = XOR(bytes 0 .. 3+Length-1), 即从帧头到Payload末尾
 *===========================================================================*/
void Debug_ProcessRxByte(uint8_t byte)
{
	rx_idle_count = 0;  /* 收到字节, 复位空闲计数(volatile, ISR/task共享) */

	switch (rx_state)
	{
	case RX_WAIT_HEADER1:
		if (byte == DEBUG_FRAME_HEADER1)
		{
			rx_checksum = byte;
			rx_state = RX_WAIT_HEADER2;
		}
		break;

	case RX_WAIT_HEADER2:
		if (byte == DEBUG_FRAME_HEADER2)
		{
			rx_checksum ^= byte;
			rx_state = RX_WAIT_CMD;
		}
		else if (byte == DEBUG_FRAME_HEADER1)
		{
			/* 当前字节可能是新帧的第一个帧头,重新开始匹配 */
			rx_checksum = byte;
			rx_state = RX_WAIT_HEADER2;
		}
		else
		{
			rx_state = RX_WAIT_HEADER1;
		}
		break;

	case RX_WAIT_CMD:
		rx_cmd = byte;
		rx_checksum ^= byte;
		rx_state = RX_WAIT_LEN;
		break;

	case RX_WAIT_LEN:
		rx_len = byte;
		rx_checksum ^= byte;
		rx_idx = 0;
		if (rx_len == 0)
		{
			rx_state = RX_WAIT_CHECKSUM;
		}
		else if (rx_len > DEBUG_RX_BUF_LEN)
		{
			rx_state = RX_WAIT_HEADER1; /* 长度超限,丢弃 */
		}
		else
		{
			rx_state = RX_WAIT_PAYLOAD;
		}
		break;

	case RX_WAIT_PAYLOAD:
		rx_buf[rx_idx++] = byte;
		rx_checksum ^= byte;
		if (rx_idx >= rx_len)
		{
			rx_state = RX_WAIT_CHECKSUM;
		}
		break;

	case RX_WAIT_CHECKSUM:
		if (byte == rx_checksum)
		{
			debug_execute_cmd(rx_cmd, rx_buf, rx_len);
		}
		rx_state = RX_WAIT_HEADER1;
		break;

	default:
		rx_state = RX_WAIT_HEADER1;
		break;
	}
}

/*===========================================================================
 * 命令执行
 *===========================================================================*/
static void debug_execute_cmd(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	float kp, ki, kd, val;
	float speed_a, speed_b;
	int16_t pwm_a, pwm_b;

	switch (cmd)
	{
	case DEBUG_CMD_SET_PID_AB: /* 设置两轮PID(相同值), payload: kp(f32)+ki(f32)+kd(f32)=12B */
		if (len >= 12)
		{
			kp = unpack_float(&payload[0]);
			ki = unpack_float(&payload[4]);
			kd = unpack_float(&payload[8]);
			if (!is_valid_pid(kp) || !is_valid_pid(ki) || !is_valid_pid(kd))
				break;
			PI_MotorA.kp = kp;  PI_MotorA.ki = ki;  PI_MotorA.kd = kd;
			PI_MotorB.kp = kp;  PI_MotorB.ki = ki;  PI_MotorB.kd = kd;
			robot.V_KP = kp;
			robot.V_KI = ki;
		}
		break;

	case DEBUG_CMD_SET_PID_A: /* 设置电机A PID */
		if (len >= 12)
		{
			kp = unpack_float(&payload[0]);
			ki = unpack_float(&payload[4]);
			kd = unpack_float(&payload[8]);
			if (!is_valid_pid(kp) || !is_valid_pid(ki) || !is_valid_pid(kd))
				break;
			PI_MotorA.kp = kp;  PI_MotorA.ki = ki;  PI_MotorA.kd = kd;
		}
		break;

	case DEBUG_CMD_SET_PID_B: /* 设置电机B PID */
		if (len >= 12)
		{
			kp = unpack_float(&payload[0]);
			ki = unpack_float(&payload[4]);
			kd = unpack_float(&payload[8]);
			if (!is_valid_pid(kp) || !is_valid_pid(ki) || !is_valid_pid(kd))
				break;
			PI_MotorB.kp = kp;  PI_MotorB.ki = ki;  PI_MotorB.kd = kd;
		}
		break;

	case DEBUG_CMD_SET_RCSPD: /* 设置rc_speed */
		if (len >= 4)
		{
			val = unpack_float(&payload[0]);
			if (val > 0 && val <= 10000.0f)
				robot_control.rc_speed = val;
		}
		break;

	case DEBUG_CMD_SET_MAXSPD: /* 设置limt_max_speed */
		if (len >= 4)
		{
			val = unpack_float(&payload[0]);
			if (val > 0 && val <= 10.0f)
				robot_control.limt_max_speed = val;
		}
		break;

	case DEBUG_CMD_SET_SMOOTH: /* 设置smooth_MotorStep */
		if (len >= 4)
		{
			val = unpack_float(&payload[0]);
			if (val > 0 && val <= 1.0f)
				robot_control.smooth_MotorStep = val;
		}
		break;

	case DEBUG_CMD_SET_SPEED_AB: /* 设置A/B两轮目标速度 */
		if (len >= 8)
		{
			speed_a = unpack_float(&payload[0]);
			speed_b = unpack_float(&payload[4]);
			if (!is_valid_speed(speed_a) || !is_valid_speed(speed_b))
				break;
			Set_UartTargetSpeed(speed_a, speed_b);
		}
		break;

	case DEBUG_CMD_SET_PWM_AB: /* 设置A/B两轮目标PWM */
		if (len >= 4)
		{
			pwm_a = unpack_int16(&payload[0]);
			pwm_b = unpack_int16(&payload[2]);
			if (pwm_a < 0 || pwm_a > FULL_DUTYCYCLE || pwm_b < 0 || pwm_b > FULL_DUTYCYCLE)
				break;
			Set_UartTargetPwm(pwm_a, pwm_b);
		}
		break;

	case DEBUG_CMD_QUERY: /* 查询当前参数 */
		param_reply_pending = 1; /* 下一个发送周期回复 */
		break;

	default:
		break;
	}
}

/*===========================================================================
 * RX超时检测 (从Balance_task 100Hz调用)
 * 检测状态机是否卡在中间状态,超过阈值则复位
 *===========================================================================*/
void Debug_CheckRxTimeout(void)
{
	if (rx_state != RX_WAIT_HEADER1)
	{
		if (++rx_idle_count > DEBUG_RX_IDLE_THRESHOLD_CYCLES)
		{
			/* 复位状态机及所有中间状态, 避免半帧残留 */
			rx_state = RX_WAIT_HEADER1;
			rx_cmd = 0;
			rx_len = 0;
			rx_idx = 0;
			rx_checksum = 0;
			rx_idle_count = 0;
		}
	}
}
