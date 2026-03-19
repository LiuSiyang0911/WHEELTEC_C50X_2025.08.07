#ifndef __ENCODER_H
#define __ENCODER_H 
#include "sys.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//���ɴ���65535����ΪSTM32F103�Ķ�ʱ����16λ��
#define ENCODER_TIM_PERIOD (u16)(65535)   

//������ö��
typedef enum{
	Encoder_A,
	Encoder_B,
	Encoder_C,
	Encoder_D
}ENCODER_t;

/*--------ENCODER Interface Fun --------*/
short Read_Encoder(ENCODER_t e);
/*----------------------------------*/

/*--------T法（周期测速）— 仅 AKM_CAR T-method, AKM_CAR only --------*/
#if defined AKM_CAR
/* 由 ISR 更新的原始速度 [0]=EncA [1]=EncB, 单位 m/s (未除WheelDiff) */
extern volatile float    encoder_T_velocity_raw[2];
/* 上次脉冲时刻, 用于零速超时判断 */
extern volatile uint32_t last_pulse_update[2];
/* 由 system.c 在 Robot_Select() 后赋值: Wheel_Circ / (Encoder_precision/4) */
extern float             enc_T_scale_base;

void     TIM6_FreeRun_Init(void);
void     Encoder_T_Method_Init(void);
uint32_t get_us_tick(void);
#endif /* AKM_CAR */
/*----------------------------------*/


/*--------ENCODER_A config--------*/
#define ENABLE_ENCODER_A_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE)
#define ENBALE_ENCODER_A1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_A2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define ENCODER_A_TIM     TIM2             //������A�ӿ���ʹ�õĶ�ʱ��

#define ENCODER_A1_PORT   GPIOA            //������A�ӿ�1�Ŷ˿�
#define ENCODER_A1_PIN    GPIO_Pin_15      //������A�ӿ�1������

#define ENCODER_A2_PORT   GPIOB            //������A�ӿ�2�Ŷ˿�
#define ENCODER_A2_PIN    GPIO_Pin_3       //������A�ӿ�2������
/*----------------------------------*/

/*--------ENCODER_A Interface Fun --------*/
void EncoderA_Init(void);
/*----------------------------------*/


/*--------ENCODER_B config--------*/
#define ENABLE_ENCODER_B_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE)
#define ENBALE_ENCODER_B1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_B2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define ENCODER_B_TIM     TIM3             //������B�ӿ���ʹ�õĶ�ʱ��

#define ENCODER_B1_PORT   GPIOA            //������B�ӿ�1�Ŷ˿�
#define ENCODER_B1_PIN    GPIO_Pin_6       //������B�ӿ�1������

#define ENCODER_B2_PORT   GPIOA            //������B�ӿ�2�Ŷ˿�
#define ENCODER_B2_PIN    GPIO_Pin_7       //������B�ӿ�2������
/*----------------------------------*/

/*--------ENCODER_B Interface Fun --------*/
void EncoderB_Init(void);
/*----------------------------------*/

/*--------ENCODER_C config--------*/
#define ENABLE_ENCODER_C_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE)
#define ENBALE_ENCODER_C1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)
#define ENBALE_ENCODER_C2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define ENCODER_C_TIM     TIM4             //������C�ӿ���ʹ�õĶ�ʱ��

#define ENCODER_C1_PORT   GPIOB            //������C�ӿ�1�Ŷ˿�
#define ENCODER_C1_PIN    GPIO_Pin_6       //������C�ӿ�1������

#define ENCODER_C2_PORT   GPIOB            //������C�ӿ�2�Ŷ˿�
#define ENCODER_C2_PIN    GPIO_Pin_7       //������C�ӿ�2������
/*----------------------------------*/

/*--------ENCODER_C Interface Fun --------*/
void EncoderC_Init(void);
/*----------------------------------*/

/*--------ENCODER_D config--------*/
#define ENABLE_ENCODER_D_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE)
#define ENBALE_ENCODER_D1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_D2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define ENCODER_D_TIM     TIM5             //������D�ӿ���ʹ�õĶ�ʱ��

#define ENCODER_D1_PORT   GPIOA            //������D�ӿ�1�Ŷ˿�
#define ENCODER_D1_PIN    GPIO_Pin_0       //������D�ӿ�1������

#define ENCODER_D2_PORT   GPIOA            //������D�ӿ�2�Ŷ˿�
#define ENCODER_D2_PIN    GPIO_Pin_1       //������D�ӿ�2������
/*----------------------------------*/

/*--------ENCODER_D Interface Fun --------*/
void EncoderD_Init(void);
/*----------------------------------*/


#endif
