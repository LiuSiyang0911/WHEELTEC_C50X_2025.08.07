#include "data_task.h"

//用于发送数据的结构体
SEND_DATA Send_Data;
#if defined AKM_CAR
SEND_AKM_EXT_DATA Send_AkmExt_Data;
#endif
SEND_AutoCharge_DATA Send_AutoCharge_Data;

/**************************************************************************
Function: Robot Data Transmission Task: Sending robot status, IMU, speed, and other information to various interfaces.
Input   : none
Output  : none
函数功能： 机器人数据发送任务,向各个接口发送机器人的状态,imu,速度等信息
入口参数：无
返回  值：无
**************************************************************************/
TaskHandle_t data_TaskHandle = NULL;

void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at DATA_TASK_RATE.
			//此任务以DATA_TASK_RATE定义的频率运行
			vTaskDelayUntil(&lastWakeTime, F2T(DATA_TASK_RATE));
			//Assign the data to be sent
			//对要进行发送的数据进行赋值
			data_transition(); 
			
//			Usart1_SendTask();
			Usart3_SendTask();
			CAN1_SendTask();
		}
}

/**************************************************************************
Functionality: Perform BCC (Block Check Character) verification on the input array and specified length
Input Parameters: Array start address, length to be verified
Return Value: BCC verification result
Author: WHEELTEC
函数功能：将传入的数组和校验的长度进行BCC校验
入口参数：数组首地址,要检验的长度
返回  值：bcc校验结果
作    者：WHEELTEC
**************************************************************************/
uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

#if defined AKM_CAR
static short Limit_Int16_FromFloat(float value)
{
	if(value > 32767.0f) return 32767;
	if(value < -32768.0f) return -32768;
	return (short)value;
}

static void Pack_U16_BE(uint8_t *buf,uint16_t value)
{
	buf[0] = value >> 8;
	buf[1] = value;
}

static void Pack_I16_BE(uint8_t *buf,int16_t value)
{
	Pack_U16_BE(buf,(uint16_t)value);
}

static void Pack_U32_BE(uint8_t *buf,uint32_t value)
{
	buf[0] = value >> 24;
	buf[1] = value >> 16;
	buf[2] = value >> 8;
	buf[3] = value;
}

static void Pack_I32_BE(uint8_t *buf,int32_t value)
{
	Pack_U32_BE(buf,(uint32_t)value);
}
#endif

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
static void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾
	
	//根据车型的不同运行相对应的运动学正解函数,获取机器人3轴计算速度
	float *vel;
	#if defined AKM_CAR || defined DIFF_CAR
		vel = Kinematics_akm_diff(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder);
	#elif defined MEC_CAR || defined _4WD_CAR
		vel = Kinematics_mec_4wd(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder,robot.MOTOR_D.Encoder);
	#elif defined OMNI_CAR
		vel = Kinematics_omni(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder);
	#endif
	
	//Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
	//运动学正解，从各车轮当前速度求出三轴当前速度
	Send_Data.Sensor_Str.Vel.X_speed = vel[0]*1000; //小车x轴速度,扩大1000倍发送
	Send_Data.Sensor_Str.Vel.Y_speed = vel[1]*1000; //小车y轴速度,扩大1000倍发送
	Send_Data.Sensor_Str.Vel.Z_speed = vel[2]*1000; //小车z轴速度,扩大1000倍发送
	
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	
	if( 0 == robot_control.FlagStop ) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;  
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = robot.voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
	Send_Data.buffer[1]=robot_control.FlagStop; //Car software loss marker //小车软件失能标志位
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.Vel.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.Vel.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Vel.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Vel.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Vel.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Vel.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22]=Check_BCC(Send_Data.buffer,22); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾

	#if defined AKM_CAR
	AkmExt_DataTransition();
	#endif
	
	///////////////////////自动回充相关变量赋值/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //帧头赋值0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//帧尾赋值0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)charger.ChargingCurrent;//充电电流赋值
	
	Send_AutoCharge_Data.AutoCharge_Str.RED = charger.RED_STATE; //红外标志位赋值
	Send_AutoCharge_Data.AutoCharge_Str.Charging = charger.Charging; //是否在充电标志位赋值

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//帧头0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//充电电流高8位
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//充电电流低8位
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//是否接收到红外标志位
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//是否在充电标志位
	Send_AutoCharge_Data.buffer[5] = charger.AllowRecharge;									//自动回充的状态
	Send_AutoCharge_Data.buffer[6] = Check_BCC(Send_AutoCharge_Data.buffer,6);				//校验位
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//帧尾0x7F
	///////////////////////自动回充相关变量赋值/////////////////////
}

#if defined AKM_CAR
/**************************************************************************
Function: Assemble AKM motion extension telemetry frame.
Input   : none
Output  : none
函数功能：组装AKM扩展运动遥测帧, 内嵌原24字节帧并追加底层观测量
入口参数：无
返回  值：无
**************************************************************************/
static void AkmExt_DataTransition(void)
{
	static uint16_t seq_id = 0;
	static uint32_t last_tick_us = 0;
	static uint8_t has_last_tick = 0;
	uint8_t *payload;
	uint8_t *buf;
	uint8_t i;
	uint16_t status_flags;
	uint32_t now_us;
	uint16_t dt_us;
	float steering_angle_rad;
	short steering_angle_x10000;

	buf = Send_AkmExt_Data.buffer;
	payload = &buf[4];
	now_us = get_us_tick();
	dt_us = has_last_tick ? (uint16_t)(now_us - last_tick_us) : 0;
	last_tick_us = now_us;
	has_last_tick = 1;
	status_flags = 0x0040u; //legacy 24B data valid //原24字节数据有效
	steering_angle_rad = 0.0f;
	steering_angle_x10000 = 0;

	if(robot_control.FlagStop) status_flags |= 0x0001u;
	if(robot_control.command_lostcount > BALANCE_TASK_RATE) status_flags |= 0x0002u;
	if(robot.voltage < 20.0f && charger.AllowRecharge == 0) status_flags |= 0x0004u;
	if(robot_check.errorflag) status_flags |= 0x0010u;

	if(robot.type >= 2 && robot.type <= 5)
	{
		steering_angle_rad = angle_to_rad(SteeringStructure_ForwardKinematic((short)robot.SERVO.Encoder));
		steering_angle_x10000 = Limit_Int16_FromFloat(steering_angle_rad * 10000.0f);
		status_flags |= 0x0020u; //steering angle calibrated from slide raw //滑轨反馈转角有效
	}

	buf[0] = AKM_EXT_FRAME_HEADER;
	buf[1] = AKM_EXT_FRAME_TYPE_MOTION;
	buf[2] = AKM_EXT_PROTOCOL_VERSION;
	buf[3] = AKM_EXT_PAYLOAD_SIZE;

	Pack_U16_BE(&payload[0],seq_id++);
	Pack_U32_BE(&payload[2],now_us);
	Pack_U16_BE(&payload[6],dt_us);

	for(i=0;i<SEND_DATA_SIZE;i++)
	{
		payload[8+i] = Send_Data.buffer[i];
	}

	Pack_I32_BE(&payload[32],akm_encoder_delta_raw[0]);
	Pack_I32_BE(&payload[36],akm_encoder_delta_raw[1]);
	Pack_I16_BE(&payload[40],Limit_Int16_FromFloat(robot.MOTOR_A.Encoder * 1000.0f));
	Pack_I16_BE(&payload[42],Limit_Int16_FromFloat(robot.MOTOR_B.Encoder * 1000.0f));
	Pack_I16_BE(&payload[44],Limit_Int16_FromFloat(robot.SERVO.Encoder));
	Pack_I16_BE(&payload[46],Limit_Int16_FromFloat(robot.SERVO.Target));
	Pack_I16_BE(&payload[48],steering_angle_x10000);
	Pack_I16_BE(&payload[50],akm_last_servo_pwm);
	Pack_I16_BE(&payload[52],akm_last_motor_pwm[0]);
	Pack_I16_BE(&payload[54],akm_last_motor_pwm[1]);
	Pack_I16_BE(&payload[56],Limit_Int16_FromFloat(robot_control.Vx * 1000.0f));
	Pack_I16_BE(&payload[58],Limit_Int16_FromFloat(robot_control.Vy * 1000.0f));
	Pack_I16_BE(&payload[60],Limit_Int16_FromFloat(robot_control.Vz * 1000.0f));
	Pack_U16_BE(&payload[62],status_flags);
	payload[64] = robot_control.ControlMode;
	payload[65] = robot.type;

	buf[AKM_EXT_FRAME_SIZE-2] = Check_BCC(buf,AKM_EXT_FRAME_SIZE-2);
	buf[AKM_EXT_FRAME_SIZE-1] = AKM_EXT_FRAME_TAIL;
}
#endif

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
#if 0
static void Usart1_SendTask(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		uart1_send(Send_Data.buffer[i]);
	}	 
	
	if(SysVal.HardWare_charger==1)
	{
		//存在回充装备时，向上层发送自动回充相关变量
		for(i=0; i<8; i++)
		{
			uart3_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
#endif

/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
static void Usart3_SendTask(void)
{
  unsigned char i = 0;	
	#if defined AKM_CAR
	for(i=0; i<AKM_EXT_FRAME_SIZE; i++)
	{
		uart3_send(Send_AkmExt_Data.buffer[i]);
	}
	#else
	for(i=0; i<24; i++)
	{
		uart3_send(Send_Data.buffer[i]);
	}	 
	#endif
	
	if(SysVal.HardWare_charger==1)
	{
		//存在回充装备时，向上层发送自动回充相关变量
		for(i=0; i<8; i++)
		{
			uart3_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}

}

/**************************************************************************
Function: CAN1 sends data
Input   : none
Output  : none
函数功能：CAN1发送数据
入口参数：无
返回  值：无
**************************************************************************/
static void CAN1_SendTask(void)
{
	u8 CAN_SENT[8],i;
	
	//24字节数据分3组发送,使用标准帧id 0x101 0x102 0x103
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//存在回充装备时，将回充相关配置数据发送到回充装备
	if(SysVal.HardWare_charger==1)
	{
		CAN_Send_AutoRecharge();
	}
}

/*-------------------- Positive kinematics correlation function for each vehicle model ------------------------*/
/*--------------------------------         各车型运动学正解相关函数          ------------------------------------*/
/**************************************************************************
Function Purpose: akm/diff Kinematic Analysis
Input Parameters: Left wheel speed, Right wheel speed, in meters per second (m/s)
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
函数功能：阿克曼/差速 运动学正解
入口参数：左轮速度、右轮速度,单位 m/s
返回  值：机器人x、y、z三轴速度
作    者：WHEELTEC
**************************************************************************/
#if defined AKM_CAR || defined DIFF_CAR
static float* Kinematics_akm_diff(float motorA,float motorB)
{
	static float vel[3];
	//xyz三轴计算速度
	#if defined AKM_CAR
	vel[0] = (motorA + motorB)/2.0f;
	vel[1] = 0;
	#else
	vel[0] = (motorA + motorB)/2.0f;
	vel[1] = 0;
	#endif
	vel[2] = (motorB - motorA)/robot.HardwareParam.WheelSpacing;
	
	return vel;
}

/**************************************************************************
Function Purpose: mec/4wd Kinematic Analysis
Input Parameters: Robot's four-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
函数功能：麦轮/四驱 运动学正解
入口参数：机器人四个轮的速度,单位 m/s
返回  值：机器人x、y、z三轴速度
作    者：WHEELTEC
**************************************************************************/
#elif defined MEC_CAR || defined _4WD_CAR
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD)
{
	static float vel[3];
	//xyz三轴计算速度
	vel[0] = (motorA+motorB+motorC+motorD)/4.0f;
	vel[1] = (motorA-motorB+motorC-motorD)/4.0f;
	vel[2] = (-motorA-motorB+motorC+motorD)/4.0f/( robot.HardwareParam.WheelSpacing + robot.HardwareParam.AxleSpacing );
	
	return vel;
}

/**************************************************************************
Function Purpose: omni Kinematic Analysis
Input Parameters: Robot's three-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: WHEELTEC
函数功能：全向轮 运动学正解
入口参数：机器人3个轮的速度,单位 m/s
返回  值：机器人x、y、z三轴速度
作    者：WHEELTEC
**************************************************************************/
#elif defined OMNI_CAR
static float* Kinematics_omni(float motorA,float motorB,float motorC)
{
	static float vel[3];
	//xyz三轴计算速度
	vel[0] = (motorC - motorB)/2.0f/robot.HardwareParam.X_PARAMETER;
	vel[1] = (motorA*2 - motorB - motorC)/3.0f;
	vel[2] = (motorA + motorB + motorC )/ 3.0f /robot.HardwareParam.TurnRadiaus;
	
	return vel;
}

#endif


