#include "encoder.h"

//ͨ�ñ�������ʼ��,������Ķ�ʱ���Լ���Ӧ���ų�ʼ��Ϊ������ģʽ3
static void Encoder_TI12_ModeInit(GPIO_TypeDef* GPIOx_1,uint16_t GPIO_PIN_1,GPIO_TypeDef* GPIOx_2,uint16_t GPIO_PIN_2,TIM_TypeDef* TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
   
	//��������
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx_2, &GPIO_InitStructure);
	
	//ȷ�ϸ��õĶ�ʱ��
	//��TIM6��7��ͨ����,��������ͨ��,��Ӧʹ�ø��ù���
	uint8_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                      GPIO_AF = 0x01;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = 0x02;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = 0x03;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = 0x09;
	
	//ȷ��1�����Ÿ��õ����ź�
	uint8_t PinSource=0;
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_1>>PinSource)&0x01 )==1 ) break;
	}
	//��������
	GPIO_PinAFConfig(GPIOx_1,PinSource,GPIO_AF);
	
	//ȷ��2�����Ÿ��õ����ź�
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (GPIO_PIN_2>>PinSource)&0x01 )==1 ) break;
	}
	//��������
	GPIO_PinAFConfig(GPIOx_2,PinSource,GPIO_AF);
	
	
	//��ʱ������
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 				    //����Ƶ
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      //�趨�������Զ���װֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //ѡ��ʱ�ӷ�Ƶ������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);    //��ʼ����ʱ��

	//ʹ�ñ�����ģʽ3
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//�˲�ϵ������Ϊ0
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);
	
	//���TIM�ĸ��±�־λ
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	//��ռ���ֵ
    TIM_SetCounter(TIMx,0);
	//������ʱ��
    TIM_Cmd(TIMx, ENABLE);
}

/**************************************************************************
Function: Encoder interface A initialization
Input   : none
Output  : none
�������ܣ��������ӿ�A��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void EncoderA_Init(void)
{
    ENABLE_ENCODER_A_TIM_CLOCK;   //ʹ�ܶ�ʱ��
	ENBALE_ENCODER_A1_PORT_CLOCK; //ʹ��1�����Ŷ�Ӧ�Ķ˿�
	ENBALE_ENCODER_A2_PORT_CLOCK; //ʹ��2�����Ŷ�Ӧ�Ķ˿�
	
	//���ñ�����A�����ڱ�����3ģʽ
	Encoder_TI12_ModeInit(ENCODER_A1_PORT,ENCODER_A1_PIN,ENCODER_A2_PORT,ENCODER_A2_PIN,ENCODER_A_TIM);
}


void EncoderB_Init(void)
{
    ENABLE_ENCODER_B_TIM_CLOCK;   //ʹ�ܶ�ʱ��
	ENBALE_ENCODER_B1_PORT_CLOCK; //ʹ��1�����Ŷ�Ӧ�Ķ˿�
	ENBALE_ENCODER_B2_PORT_CLOCK; //ʹ��2�����Ŷ�Ӧ�Ķ˿�
	
	//���ñ�����B�����ڱ�����3ģʽ
	Encoder_TI12_ModeInit(ENCODER_B1_PORT,ENCODER_B1_PIN,ENCODER_B2_PORT,ENCODER_B2_PIN,ENCODER_B_TIM);
}


void EncoderC_Init(void)
{
    ENABLE_ENCODER_C_TIM_CLOCK;   //ʹ�ܶ�ʱ��
	ENBALE_ENCODER_C1_PORT_CLOCK; //ʹ��1�����Ŷ�Ӧ�Ķ˿�
	ENBALE_ENCODER_C2_PORT_CLOCK; //ʹ��2�����Ŷ�Ӧ�Ķ˿�
   
	//���ñ�����C�����ڱ�����3ģʽ
	Encoder_TI12_ModeInit(ENCODER_C1_PORT,ENCODER_C1_PIN,ENCODER_C2_PORT,ENCODER_C2_PIN,ENCODER_C_TIM);
}


void EncoderD_Init(void)
{
    ENABLE_ENCODER_D_TIM_CLOCK;   //ʹ�ܶ�ʱ��
	ENBALE_ENCODER_D1_PORT_CLOCK; //ʹ��1�����Ŷ�Ӧ�Ķ˿�
	ENBALE_ENCODER_D2_PORT_CLOCK; //ʹ��2�����Ŷ�Ӧ�Ķ˿�
   
	//���ñ�����D�����ڱ�����3ģʽ
	Encoder_TI12_ModeInit(ENCODER_D1_PORT,ENCODER_D1_PIN,ENCODER_D2_PORT,ENCODER_D2_PIN,ENCODER_D_TIM);
}


/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
�������ܣ���ȡ����������
��ڲ�������ʱ��
����  ֵ����������ֵ(�����ٶ�)
**************************************************************************/
short Read_Encoder(ENCODER_t e)
{
    short Encoder_TIM;
    switch(e)
    {
		case Encoder_A:
			Encoder_TIM = (short)ENCODER_A_TIM -> CNT;
			ENCODER_A_TIM -> CNT=0;
			break;
		case Encoder_B:
			Encoder_TIM = (short)ENCODER_B_TIM -> CNT;
			ENCODER_B_TIM -> CNT=0;
			break;
		case Encoder_C:
			Encoder_TIM = (short)ENCODER_C_TIM -> CNT;
			ENCODER_C_TIM -> CNT=0;
			break;
		case Encoder_D:
			Encoder_TIM = (short)ENCODER_D_TIM -> CNT;
			ENCODER_D_TIM -> CNT=0;
			break;
		default:
			Encoder_TIM=0;
    }
    return Encoder_TIM;
}


/*===========================================================================
 * T法（周期测速）— 仅用于 AKM_CAR（2路编码器）
 * T-method speed measurement, AKM_CAR only (2 encoders)
 * EncA: PB3/EXTI3 (TIM2 CH2),  EncB: PA6/EXTI6 (TIM3 CH1)
 * 时间戳由 TIM6 提供, 1MHz (1μs/tick), 32位有效范围约71分钟
 *=========================================================================*/
#if defined AKM_CAR

/* TIM6 溢出高16位计数器 — TIM6_DAC_IRQHandler 中自增 */
static volatile uint32_t tim6_high = 0;

/* ISR 写, Get_Robot_FeedBack 读: 带方向的原始速度(m/s, 未除WheelDiff) */
volatile float  encoder_T_velocity_raw[2] = {0.0f, 0.0f};

/* 由 system.c 在 Robot_Select() 后赋值: Wheel_Circ / (Encoder_precision/4) */
float enc_T_scale_base = 1.0f;

/* 上次脉冲时刻(us), 用于计算 dt */
static volatile uint32_t last_pulse_time[2] = {0, 0};

/* 上次脉冲更新时刻, 用于零速超时检测 */
volatile uint32_t last_pulse_update[2] = {0, 0};


/**************************************************************************
Function: Get 32-bit microsecond timestamp from TIM6 free-run counter
Input   : none
Output  : timestamp in microseconds
函数功能：从TIM6自由运行计数器获取32位微秒时间戳
输入参数：无
返 回 值：微秒时间戳
**************************************************************************/
uint32_t get_us_tick(void)
{
    uint32_t hi, lo;
    /* 双读校验: 防止 TIM6 在两次读取之间发生溢出导致数值错误 */
    do {
        hi = tim6_high;
        lo = TIM6->CNT;
    } while (hi != tim6_high);
    return (hi << 16) | lo;
}


/**************************************************************************
Function: Initialize TIM6 as 1MHz free-running counter for T-method
Input   : none
Output  : none
函数功能：将TIM6初始化为1MHz自由运行计数器，用于T法时间戳
**************************************************************************/
void TIM6_FreeRun_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef        NVIC_InitStruct;

    /* TIM6 时钟使能 (APB1 = 84MHz) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /* PSC=83 → 84MHz/84 = 1MHz (1μs/tick), ARR=65535 → 16bit */
    TIM_TimeBaseStruct.TIM_Prescaler     = 83;
    TIM_TimeBaseStruct.TIM_Period        = 65535;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

    /* 使能溢出更新中断 */
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    /* NVIC: 优先级0 (最高), 确保可以抢占EXTI中断, 保证时间戳一致性 */
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* 启动计数器 */
    TIM_Cmd(TIM6, ENABLE);
}


/**************************************************************************
Function: Configure EXTI for T-method edge detection on encoder pins
          PB3 -> EXTI3 (EncA),  PA6 -> EXTI6 (EncB)
Input   : none
Output  : none
函数功能：为T法配置编码器引脚上升沿EXTI中断
         PB3→EXTI3(EncA, TIM2 CH2),  PA6→EXTI6(EncB, TIM3 CH1)
**************************************************************************/
void Encoder_T_Method_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 使能 SYSCFG 时钟 (EXTI 复用需要) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* ---- EncA: PB3 → EXTI3 ---- */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line3;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = EXTI3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* ---- EncB: PA6 → EXTI6 (共享 EXTI9_5 处理器) ---- */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);
    EXTI_InitStruct.EXTI_Line = EXTI_Line6;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStruct);
}


/**************************************************************************
Function: TIM6 overflow interrupt — increments high 16 bits of timestamp
函数功能：TIM6溢出中断，递增时间戳高16位
**************************************************************************/
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        tim6_high++;
    }
}


/**************************************************************************
Function: EncA rising edge interrupt (PB3/EXTI3) — T-method velocity update
函数功能：编码器A上升沿中断(PB3/EXTI3)，更新T法速度
         方向来自 TIM2->CR1 bit4 (DIR): 0=正转, 1=反转
**************************************************************************/
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3)) {
        uint32_t now = get_us_tick();
        uint32_t dt  = now - last_pulse_time[0];

        /* 100μs < dt < 500ms 为有效范围 */
        if (dt > 100u && dt < 500000u) {
            /* TIM2 DIR bit: 0=计数递增(正转), 1=计数递减(反转) */
            float sign = ((TIM2->CR1 >> 4) & 1u) ? -1.0f : 1.0f;
            encoder_T_velocity_raw[0] = sign * (1000000.0f / (float)dt) * enc_T_scale_base;
        }
        last_pulse_time[0]   = now;
        last_pulse_update[0] = now;
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}


/**************************************************************************
Function: EncB rising edge interrupt (PA6/EXTI9_5) — T-method velocity update
函数功能：编码器B上升沿中断(PA6/EXTI9_5)，更新T法速度
         方向来自 TIM3->CR1 bit4 (DIR), 符号与EncA相反(对应M法中的-Encoder_B_pr)
         TIM3 DIR: 0=计数递增, 此时对应 robot.MOTOR_B 负方向
**************************************************************************/
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line6)) {
        uint32_t now = get_us_tick();
        uint32_t dt  = now - last_pulse_time[1];

        if (dt > 100u && dt < 500000u) {
            /* EncB 方向取反: 与 M 法 -Encoder_B_pr 保持一致 */
            float sign = ((TIM3->CR1 >> 4) & 1u) ? 1.0f : -1.0f;
            encoder_T_velocity_raw[1] = sign * (1000000.0f / (float)dt) * enc_T_scale_base;
        }
        last_pulse_time[1]   = now;
        last_pulse_update[1] = now;
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

#endif /* AKM_CAR */

