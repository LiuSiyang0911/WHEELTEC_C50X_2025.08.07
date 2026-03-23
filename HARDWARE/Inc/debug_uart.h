#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H
#include "sys.h"

/*
 * 调试串口接口 - 通过UART1 DMA发送实时调试数据
 *
 * TX数据帧 (100Hz, 32字节): 编码器原始速度、Kalman滤波速度、目标速度、PID输出
 * TX参数帧 (按需, 40字节): PID参数、速度限制、平滑步进
 * RX命令帧: 设置PID参数、速度限制等
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

/* RX命令ID */
#define DEBUG_CMD_SET_PID_AB 0x10  /* 设置两轮PID(相同值) */
#define DEBUG_CMD_SET_PID_A  0x11  /* 设置电机A PID */
#define DEBUG_CMD_SET_PID_B  0x12  /* 设置电机B PID */
#define DEBUG_CMD_SET_RCSPD  0x20  /* 设置rc_speed */
#define DEBUG_CMD_SET_MAXSPD 0x21  /* 设置limt_max_speed */
#define DEBUG_CMD_SET_SMOOTH 0x22  /* 设置smooth_MotorStep */
#define DEBUG_CMD_QUERY      0x30  /* 查询当前参数 */

/* 帧长度 */
#define DEBUG_DATA_FRAME_LEN  32  /* TX数据帧总长 */
#define DEBUG_PARAM_FRAME_LEN 40  /* TX参数帧总长 */
#define DEBUG_RX_BUF_LEN      24  /* RX接收缓冲区(最长payload=12B + 开销) */

/* 对外接口 */
void Debug_UART_DMA_Init(void);
void Debug_SendDataFrame(void);
void Debug_SendParamFrame(void);
void Debug_ProcessRxByte(uint8_t byte);
void Debug_CheckRxTimeout(void);
void Debug_WaitTxDone(void);

#endif
