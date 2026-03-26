#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H
#include "sys.h"

/*
 * 调试串口接口 - 通过UART1 DMA发送实时调试数据
 *
 * TX数据帧 (Frame 0x01): T法原始速度、M法原始速度、融合后速度、目标速度、输出
 * TX参数帧 (Frame 0x02): 控制模式、LADRC参数、速度限制、平滑步进
 * TX控制状态帧 (Frame 0x03): 测速反馈、ESO状态、前馈输出、异常计数
 * RX命令帧: 设置PID/LADRC参数、速度限制、控制模式等
 *
 * 协议帧头: 0xAA 0x55
 * 字节序: 小端序 (ARM Cortex-M4 native)
 * 校验: XOR (从帧头到payload末尾)
 */

/* 帧头 */
#define DEBUG_FRAME_HEADER1  0xAA
#define DEBUG_FRAME_HEADER2  0x55

/* TX帧ID */
#define DEBUG_FRAME_DATA     0x01  /* 100Hz数据帧 */
#define DEBUG_FRAME_PARAM    0x02  /* 参数回复帧 */
#define DEBUG_FRAME_CTRL     0x03  /* 控制状态帧 */

/* RX命令ID */
#define DEBUG_CMD_SET_PID_AB 0x10  /* 设置两轮PID(相同值) */
#define DEBUG_CMD_SET_PID_A  0x11  /* 设置电机A PID */
#define DEBUG_CMD_SET_PID_B  0x12  /* 设置电机B PID */
#define DEBUG_CMD_SET_RCSPD  0x20  /* 设置rc_speed */
#define DEBUG_CMD_SET_MAXSPD 0x21  /* 设置limt_max_speed */
#define DEBUG_CMD_SET_SMOOTH 0x22  /* 设置smooth_MotorStep */
#define DEBUG_CMD_SET_SPEED_AB 0x23  /* 设置A/B两轮目标速度 */
#define DEBUG_CMD_SET_PWM_AB   0x24  /* 设置A/B两轮目标PWM */
#define DEBUG_CMD_QUERY      0x30  /* 查询当前参数 */
#define DEBUG_CMD_SET_CTRL_MODE 0x40 /* 设置控制模式 */
#define DEBUG_CMD_SET_LADRC_AB  0x41 /* 设置两轮LADRC参数 */

/* 帧长度 */
#define DEBUG_DATA_FRAME_LEN  40  /* TX数据帧总长 */
#define DEBUG_PARAM_FRAME_LEN 45  /* TX参数帧总长 */
#define DEBUG_CTRL_FRAME_LEN  45  /* TX控制状态帧总长 */
#define DEBUG_RX_BUF_LEN      32  /* RX接收缓冲区(最长payload=20B + 开销) */

/* 对外接口 */
void Debug_UART_DMA_Init(void);
void Debug_SendDataFrame(void);
void Debug_SendParamFrame(void);
void Debug_ProcessRxByte(uint8_t byte);
void Debug_CheckRxTimeout(void);
void Debug_WaitTxDone(void);

#endif
