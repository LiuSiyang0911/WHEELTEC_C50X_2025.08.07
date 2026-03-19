/***********************************************
��˾����ݸ��΢�����ܿƼ����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V3.5
�޸�ʱ�䣺2021-01-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V3.5
Update��2021-01-29

All rights reserved
***********************************************/

#include "system.h"

//ϵͳ��ر���
SYS_VAL_t SysVal;

void systemInit(void)
{
	//================= General Hardware Initialization Section =================//
	//================= ͨ��Ӳ����ʼ������ =================//
    //Interrupt priority group setting
    //�ж����ȼ���������
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //Delay function initialization
    //��ʱ������ʼ��
    delay_init(168);
	
	//Prioritize initializing IIC and IMU C50C board distinguishes between new and old versions based on IMU model
	//���ȳ�ʼ��IIC��IMU.C50C��ͨ��IMU�ͺ������°���ɰ�
    //IIC initialization
    //IIC��ʼ��
    I2C_GPIOInit();
	
	//ϵͳ�������������ʼ��
	SYS_VAL_t_Init(&SysVal);
	
    //Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //����1��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
    UART1_Init(115200);
	
	//���IMUΪMPU6050,���Ǿɰ�C50C
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		
		//�ɰ�C50CӲ����ʼ��
		V1_0_LED_Init();
		V1_0_CAN1_Mode_Init(1,3,3,6,0);
		V1_0_MiniBalance_PWM_Init(16799,0);
		
		//Initialize the hardware interface to the PS2 controller
		//��ʼ����PS2�ֱ����ӵ�Ӳ���ӿ�
		PS2_Init();
		
		//PS2����������ʼ��
		PS2_Key_Param_Init();
		
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//MPU6050��ʼ�������ڶ�ȡС��������ٶȡ�������ٶ���Ϣ
		MPU6050_initialize();
	}
	
	//���IMU�ͺ�ΪICM20948,�����°�C50C
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//��ȡICM20948 id
	{
		SysVal.HardWare_Ver = V1_1;
		
		//Initialize the hardware interface connected to the LED lamp
		//��ʼ����LED�����ӵ�Ӳ���ӿ�
		RGB_LightStrip_Init();
		
		//Initialize the CAN communication interface
		//CANͨ�Žӿڳ�ʼ��
		CAN1_Mode_Init(1,3,3,6,0);
		
		//Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
		//��ʼ������ٶȿ����Լ������ڿ��Ƶ���ٶȣ�PWMƵ��10KHZ
		MiniBalance_PWM_Init(16799,0);  //�߼���ʱ��TIM8��Ƶ��Ϊ168M����PWMΪ16799��Ƶ��=168M/((16799+1)*(0+1))=10k
	
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//ICM20948��ʼ�������ڶ�ȡС��������ٶȡ�������ٶ���Ϣ
		invMSInit();
		
		//USB PS2��ʼ��
		MX_USB_HOST_Init();//����usb�ֱ�����
	}
	else //�޷�ʶ���������,��λϵͳ
	{
		NVIC_SystemReset();
	}
	
    //Initialize the hardware interface connected to the buzzer
    //��ʼ������������ӵ�Ӳ���ӿ�
    Buzzer_Init();
    
    //Initialize the hardware interface connected to the enable switch
    //��ʼ����ʹ�ܿ������ӵ�Ӳ���ӿ�
    EnableKey_Init();

    //Initialize the hardware interface connected to the user's key
    //��ʼ�����û��������ӵ�Ӳ���ӿ�
    KEY_Init();
	
    //Initialize the hardware interface connected to the OLED display
    //��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�
    OLED_Init();

	//TODO:�ָ�
    //Serial port 4 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //����4��ʼ����ͨ�Ų�����9600������������APP��ͨ��
	#if VECT_TAB_OFFSET == 0x10000 //�����Ƿ�����������¼��ѡ�������Ĳ�����
		UART4_Init(230400);
	#elif VECT_TAB_OFFSET == 0
		UART4_Init(9600);
	#endif
	
    //Serial port 3 is initialized and the baud rate is 115200.
    //Serial port 3 is the default port used to communicate with ROS terminal
    //����3��ʼ����ͨ�Ų�����115200������3ΪĬ��������ROS��ͨ�ŵĴ���
    UART3_Init(115200);

    //Initialize the model remote control interface
    //��ʼ����ģң�ؽӿ�
    Remoter_Init();
	
    //Encoder A is initialized to read the real time speed of motor A
    //������A��ʼ�������ڶ�ȡ���A��ʵʱ�ٶ�
    EncoderA_Init();
    //Encoder B is initialized to read the real time speed of motor B
    //������B��ʼ�������ڶ�ȡ���B��ʵʱ�ٶ�
    EncoderB_Init();
	
    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC���ų�ʼ�������ڶ�ȡ��ص�ѹ���λ����λ����λ����λ����С���������С�������ͺ�
    ADC1_Init();
	
	//����������ʹ��ADC2,��ʹ�ñ�����C��D;�������ͷ�֮
	#if defined AKM_CAR
		ADC2_Init();
	#else
		//Encoder C is initialized to read the real time speed of motor C  
		//������C��ʼ�������ڶ�ȡ���C��ʵʱ�ٶ�	
		EncoderC_Init();
		//Encoder D is initialized to read the real time speed of motor D
		//������D��ʼ�������ڶ�ȡ���D��ʵʱ�ٶ�	
		EncoderD_Init();  
	#endif
	
	//================= ����������ʼ������ =================//
	
	//ȷ���������ͺ�,��ʼ�������˻�е������PID����.
	Robot_Select(); 
	
	//�����˿�����ر�����ʼ��,����ң���ٶȻ�׼������ٶ����ơ��ٶ�ƽ��ϵ��������.
	ROBOT_CONTROL_t_Init(&robot_control); 
	
	//4��PI��������ʼ��
	PI_Controller_Init(&PI_MotorA,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorB,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorC,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorD,robot.V_KP,robot.V_KI);

	//T法编码器初始化 (仅AKM_CAR): TIM6自由运行计数器 + EXTI边沿检测
	//T-method encoder init (AKM_CAR only): TIM6 free-run counter + EXTI edge detect
	#if defined AKM_CAR
		//TIM6: 1MHz自由运行计数器, 提供微秒级时间戳
		TIM6_FreeRun_Init();
		//EXTI: PB3→EXTI3(EncA), PA6→EXTI6(EncB), 上升沿触发T法测速
		Encoder_T_Method_Init();
		//预计算T法速度比例系数: Wheel_Circ / (Encoder_precision / 4)
		//T-method scale: Wheel_Circ / pulses_per_wheel_rev (single-edge count)
		enc_T_scale_base = robot.HardwareParam.Wheel_Circ / (robot.HardwareParam.Encoder_precision / 4.0f);
	#endif

	//�Զ��س��豸����������ʼ��
	auto_recharge_reset();
	
	//OLED����������ʼ��
	OLED_Param_Init(&oled);
	
	//APP����������ʼ��
	APPKey_Param_Init(&appkey);
	
	//��ģң������������ʼ��
	Remoter_Param_Init(&remoter);
	
	//���������ʼ��
	Akm_ServoParam_Init(&Akm_Servo);
	
	//��Flash�����������,����������ʹ��Ĭ�ϳ�ʼ��ֵ
	FlashParam_Read();    
	
	//���������ͶԶ����ʼ��
	#if defined AKM_CAR
	
	//���䰢�������Ͷ����ʼ��
	Servo_Senior_Init(10000-1,168-1,Akm_Servo.Mid);
	robot.SERVO.Output = Akm_Servo.Mid;
	
	//���䰢�������Ͷ����ʼ��
	if( robot.type >= 2 && robot.type!= 9 )
	{
		//�ȴ�DMA�ɼ�����
		delay_ms(200);
		
		//��ȡһ�黬������,��������λ��.�ٽ������λ����ΪPWMֵ��ʼ��,�ɱ�����ͻȻ���ٹ�λ.
		short TmpPWM = get_ServoPWM( get_DMA_SlideRes() );
		
		//���䰢�������Ͷ����ʼ��,����ƫ��ֵ,���������ٸ�λ
		Servo_Top_Init(10000-1,84-1, TmpPWM );
		
		//���PI��������ʼ��.ע�������PID�����������޸�.
		PI_Controller_Init(&PI_Servo,0,0);
		
		//���ö��PI���ƻ�׼ֵ,����ƫ��ֵ,����ս���PI����ʱ�������
		PI_Servo.Output =  TmpPWM;
		
		//����ٶ�ƽ��ֵ
		robot_control.smooth_Servo = TmpPWM;
		
		//���õ��ٶ��ģʽ,�ö��������λ
		robot_control.ServoLow_flag = 1;
	}
	

	#endif
	
	//������Ӳ���豸��ʼ�����,ʹ�÷�������ʾ����rtos
	Buzzer_AddTask(1,100);//����1��,ʱ��1000ms
}


