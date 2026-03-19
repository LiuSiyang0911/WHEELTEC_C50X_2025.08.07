#include "balance_task.h"
#include "filter.h"

//��С���������
ROBOT_CONTROL_t robot_control;

//��С���Լ���ر���
ROBOT_SELFCHECK_t robot_check;

//������פ��ģʽ��ر���
static ROBOT_PARKING_t park;

//����ʽPI������
PI_CONTROLLER PI_MotorA,PI_MotorB,PI_MotorC,PI_MotorD,PI_Servo;

//���������˲�������

// Ϊ�����(���A)�����˲���
//Kalman_TypeDef Kalman_MotorA = {
//    .lastP = 0.01, .Q = 0.005, .R = 0.5, .out = 0
//};

//// Ϊ�Һ���(���B)�����˲���
//Kalman_TypeDef Kalman_MotorB = {
//    .lastP = 0.01, .Q = 0.005, .R = 0.5, .out = 0
//};

// Ϊ�����(���A)�����˲��� (����Q��R��ʹ��̬����������kgԼΪ0.42�������Ӧ��ƽ��)
Kalman_TypeDef Kalman_MotorA = {
    .lastP = 0.01, .Q = 6.5e-6f, .R = 1.11e-3f, .out = 0
};

// Ϊ�Һ���(���B)�����˲���
Kalman_TypeDef Kalman_MotorB = {
    .lastP = 0.01, .Q = 6.5e-6f, .R = 1.11e-3f, .out = 0
};

#if defined AKM_CAR
AKM_SERVO_UNLOCK_t ServoState;//���������������
#endif
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
	
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //��������100Hz��Ƶ�����У�10ms����һ�Σ�
        vTaskDelayUntil(&lastWakeTime, F2T(BALANCE_TASK_RATE));

        //Time count is no longer needed after 30 seconds
        //ʱ�������30�������Ҫ
        if(SysVal.Time_count<3000) SysVal.Time_count++;
		
        //and convert to transposition international units
        //��ȡ���������ݷ�����������ݷ���
        Get_Robot_FeedBack();
//			  any_printf(USART1, "%.4f\r\n", robot.MOTOR_B.Encoder);
		
		//�Զ��س��豸��״̬����ģң�������źż��
		Charger_DevCheck();
		
        //Function to Scan User Key Press Status
        //ɨ���û�����״̬
        UserKey_Scan( BALANCE_TASK_RATE );
		
		//Robot parking status monitoring, clearing residual motor control signals. \
		  Reduce power consumption, reduce motor noise.
		//������פ��״̬���,���������������.���͹���,���͵������.
		Robot_ParkingCheck();
		
		//�������ϵ��Լ�
        //�������ǳ�ʼ����ɺ�,���������ͺ��Ƿ�ѡ�����
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
        if(CONTROL_DELAY<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+200)
        {
            Drive_Motor(0.2f,0,0);//����ǰ������
			//TODO:�Լ���Ҫ���Ը��ֳ����Լ�
            robot_mode_check();  //Detection function //��⺯��
        }
        else if(CONTROL_DELAY+200<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+230)
        {
			robot_check.check_end = 1;
            Drive_Motor(0,0,0); //The stop forward control is completed //������ֹͣǰ������
			SysVal.LED_delay = 500;
        }
		
		//After the self-check is completed, obtain the control commands for the robot.
        //�Լ�����󣬻�ȡ�����˵Ŀ�������
        if(SysVal.Time_count>CONTROL_DELAY+230)
        {
			if( 0 == SysVal.SecurityLevel )//0Ϊ��߰�ȫ�ȼ�,���л����˿������ʧ����
			{
				//TODO:��ⶪʧ,��ͬ���Ʒ�ʽ,��ⶪʧ��Ƶ�ʲ�ͬ,��ʵ����ЧΪ׼.
				//uart,ros,can,usb ps2,��׿app ����,ƻ��app�޷�ʹ��,��ģ��Ҫ����ʵ��
				//TODO:��ȫ�ȼ�ֻ��uart,ros,can��Ч. ps2��app����ģ�źŶ�ʧ��Ҫ������ֹͣ
				robot_control.command_lostcount++;
				if( robot_control.command_lostcount>BALANCE_TASK_RATE )
					robot_control.Vx = 0 , robot_control.Vy = 0 , robot_control.Vz = 0;
			}
			
			//�Զ��س�ģʽ�µĿ�������.
			if(charger.AllowRecharge==1)
			{	
				#if defined AKM_CAR //���������ͻس�����߼�
					if( charger.NavWalk ) Drive_Motor(charger.Up_MoveX,charger.Up_MoveY,charger.Up_MoveZ);
					else
					{	
						if( charger.RED_STATE!=0 ) Drive_Motor(charger.Red_MoveX,charger.Red_MoveY,charger.Red_MoveZ); 
						else Drive_Motor(0,0,0); 
					}
				#else //�ǰ��������͹��ûس��߼�
					//��������˵����س䣬ͬʱû�н��յ������źţ�����������λ���ĵĻس��������
					if      (charger.NavWalk==1 && charger.RED_STATE==0) Drive_Motor(charger.Up_MoveX,charger.Up_MoveY,charger.Up_MoveZ); 
					//���յ��˺����źţ��������Իس�װ���Ļس��������
					else if (charger.RED_STATE!=0) charger.NavWalk = 0,Drive_Motor(charger.Red_MoveX,charger.Red_MoveY,charger.Red_MoveZ); 
					//��ֹû�к����ź�ʱС���˶�
					if (charger.NavWalk==0&&charger.RED_STATE==0) Drive_Motor(0,0,0); 
				#endif

			}
			//����ģʽ��������
			else
			{
				if      ( Get_Control_Mode(_APP_Control) )    Get_APPcmd();      //Handle the APP remote commands //����APPң������
				else if ( Get_Control_Mode(_RC_Control)  )    Remote_Control();  //Handle model aircraft remote commands //������ģң������
				else if (Get_Control_Mode(_PS2_Control)  )    PS2_control();     //Handle PS2 controller commands //����PS2�ֱ���������
				
				//CAN, Uart x control can directly get the 3 axis target speed, 
				//without additional processing
				//CAN������x ����ֱ�ӵõ�3��Ŀ���ٶȣ�������⴦��
				else    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
			}
        }
		
		//����Լ�ģʽ,���û���Ҫʱ����
		if( 1 == robot_check.DeepCheck && SysVal.Time_count>CONTROL_DELAY+300)
		{
			if( 1 == Deep_SelfCheck( BALANCE_TASK_RATE ) )
			{
				robot_check.DeepCheck = 0;
			}
			continue;
		}
		
		//Robot Operation Status Inspection. Inspection Items: Check if the voltage is too low, \
		if there are any self-test errors, and if the emergency stop switch has been pressed.   \
		The robot will not be allowed to be controlled if any of these conditions are not met.
		//����������״̬���.������ݣ���ѹ�Ƿ���͡��Լ��Ƿ��������ͣ�����Ƿ񱻰���.����������ʱ�������������˱�����
		robot_control.FlagStop = Turn_Off();
		
		#if defined AKM_CAR
			Servo_UnLock_Check(robot_control.FlagStop);//���������ģʽ���
		#endif
		
		//����������������
		if( 0 == robot_control.FlagStop )
		{
			//ִ�п���,���ݳ��͡��ͺŲ�ͬ,���в�ͬ�Ŀ���.
			ResponseControl();
		}
		else //ʧ�ܿ���.��տ�����.ʹ�ý���ķ�ʽ
		{
			UnResponseControl(UN_LOCK);
		}
		
		
		//����FLash����,���ݰ�����������������ƫϵ����
		uint8_t flash_check = 0;
		flash_check = FlashParam_Save( &appkey.ParamSaveFlag ); //����Flash����
		if( 1 == flash_check ) Buzzer_AddTask(1,100);      //����ɹ�,����������ʾ1��
		else if( 1 < flash_check ) Buzzer_AddTask(10,11); //����ʧ��,���ٷ���10������

    }
}

/*-------------------------------- Functions Related to PI Controller ------------------------------------*/
/*--------------------------------           PI ��������غ���          ------------------------------------*/
/**************************************************************************
Functionality: Set PID Control Parameters - Configures the proportional (Kp) and integral (Ki) parameters for PID control.
Input Parameters: Kp parameter, Ki parameter.
Return Value: None.
Author: WHEELTEC
�������ܣ�����PID���ƵĲ���
��ڲ�����Kp������Ki����
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void PI_SetParam(PI_CONTROLLER* p,int kp,int ki,int kd)
{
	//PID��������Ϊ��,Ҳ����Ϊ0.
	if( kp >= 0 ) 
	{
		p->kp = kp;
	}
	if( ki >= 0 ) 
	{
		p->ki = ki;
	}
	if( kd >= 0 ) 
	{
		p->ki = ki;
	}
}

void Set_Robot_PI_Param(int kp,int ki,int kd)
{
	if( kp >= 0 ) robot.V_KP = kp;
	if( ki >= 0 ) robot.V_KI = ki;
	
	//������·pi����
	PI_SetParam(&PI_MotorA,robot.V_KP,robot.V_KI,kd);
	PI_SetParam(&PI_MotorB,robot.V_KP,robot.V_KI,kd);
	PI_SetParam(&PI_MotorC,robot.V_KP,robot.V_KI,kd);
	PI_SetParam(&PI_MotorD,robot.V_KP,robot.V_KI,kd);
}

/**************************************************************************
Functionality: Incremental PI Controller Reset - Resets an incremental PI (Proportional-Integral) controller to its initial state.
Input Parameters: PI controller.
Return Value: None.
Author: WHEELTEC
�������ܣ�����ʽPI��������λ
��ڲ�����PI������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void PI_Controller_Reset(PI_CONTROLLER *p)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->LastestBias = 0;
	p->Output = 0;
}


/**************************************************************************
Functionality: Incremental PI Control - Implements the control logic for an 
               incremental PI (Proportional-Integral) controller using feedback and target values.
Input Parameters: PI controller, feedback value, target value.
Return Value: Control output result.
Author: WHEELTEC
�������ܣ�����ʽPI����
��ڲ�����PI������,����ֵ,Ŀ��ֵ
����  ֵ��������������
��    �ߣ�WHEELTEC
**************************************************************************/
static int Incremental_MOTOR(PI_CONTROLLER* p,float current,float target)
{
	int Output;
	//����ƫ��
	p->Bias = target - current;
	
	//�������
	Output = p->kp * ( p->Bias - p->LastBias ) + p->ki * p->Bias + p->kd *(p->Bias - 2 * p->LastBias + p->LastestBias);
//	p->Output += 0 * ( p->Bias - p->LastBias ) + 0 * p->Bias;
	p->Output = p->Output + Output;
	//����޷�
//  if (Output > 2000) p->Output = p->Output + 2000;
//  else if (Output < -2000) p->Output = p->Output - 2000;
//	else p->Output = p->Output + Output;
	
	if( p->Output >   FULL_DUTYCYCLE ) p->Output =   FULL_DUTYCYCLE;
	if( p->Output < - FULL_DUTYCYCLE ) p->Output = - FULL_DUTYCYCLE;
	
	//���汾��ƫ��
	p->LastestBias = p->LastBias;
	p->LastBias = p->Bias;
	
	
	//���
	return p->Output;
}

/**************************************************************************
Functionality: Incremental PI Control - Implements the control logic for an 
               incremental PI (Proportional-Integral) controller using feedback and target values.
Input Parameters: PI controller, feedback value, target value.
Return Value: Control output result.
Author: WHEELTEC
�������ܣ�����ʽPI����
��ڲ�����PI������,����ֵ,Ŀ��ֵ
����  ֵ��������������
��    �ߣ�WHEELTEC
**************************************************************************/
#if defined AKM_CAR
static int Incremental_Servo(PI_CONTROLLER* p,float current,float target)
{
	//�������PID����,�������û��޸�.�ѵ���Ϊ���ֵ
	float servo_kp = 0;
	float servo_ki = 0;
	int8_t servo_dir = 1;
	
	//���ݻ�����г̡���ͬ�ĳ��� ��ȷ��PI����
	if( robot.type==2 || robot.type==3 ) //�����ʽ������
	{
		//���е�ṹԭ��,��ת���г̱Ƚϳ�,��ʱ��Ҫ�ӿ�ת���ٶ�
		if( target<0 || current<-200 ) servo_kp = 0.002f*1.3f , servo_ki = 0.006f*1.3f;
		else                           servo_kp = 0.002f , servo_ki = 0.006f;
	}
	else if( robot.type==4 || robot.type==5 ) //�������������
	{
		servo_dir = -1;//�����װ,ִ�з����б仯
		
		//���е�ṹԭ��,��ת���г̱Ƚϳ�,��ʱ��Ҫ�ӿ�ת���ٶ�
		if( target>0 || current<200 )  servo_kp = 0.002f*1.3f , servo_ki = 0.006f*1.3f;
		else                           servo_kp = 0.002f , servo_ki = 0.006f;
	}
	
	//���ٶ��ģʽ
	static uint16_t low_speedMode = 0;
	if( robot_control.ServoLow_flag ) 
	{
		servo_kp = 0.001,servo_ki = 0.003;
		low_speedMode++;
		if( low_speedMode > BALANCE_TASK_RATE*2 ) //�����Զ��˳�ʱ��,2��
		{
			low_speedMode = 0;
			robot_control.ServoLow_flag = 0;//�Զ��˳�����ģʽ
		}
	}
	else
		low_speedMode = 0;
	
	//����ƫ��
	p->Bias = target - current ;
	
	//�������
	p->Output += (servo_kp*servo_dir) * ( p->Bias - p->LastBias ) + (servo_ki*servo_dir) * p->Bias;
	
	//����޷�
	if( p->Output > Akm_Servo.Max )  p->Output = Akm_Servo.Max;
	if( p->Output < Akm_Servo.Min )  p->Output = Akm_Servo.Min;
	
	//���汾��ƫ��
	p->LastBias = p->Bias;
	
	//���������ģʽʵ��.ͨ�����ж����������־λ�ɽ��÷�����ģʽ
	static uint16_t count = 0;
	if( robot_control.smooth_Servo == p->Output && robot_control.FlagStop==0 ) //���ڻ�������������ʱʹ�ñ��߼�
	{
		count++;
		if( count>=BALANCE_TASK_RATE )
		{
			count=0;
			ServoState.UnLock = 1;           //���벻����ģʽ
			ServoState.UnLock_Pos =  current;//����¼�����ǰ��λ��
			ServoState.UnLock_Target = target - Akm_Servo.Bias;//��¼�����Ŀ��λ��
			ServoState.UnLock_Output = robot.SERVO.Output;//��¼�����ǰ��pwmֵ
			return 0;//�������Ŀ��ǶȺ�,������
		}
	}
	else count = 0;
	
	#if 1
	//���ƽ������,���ͼ��ٶ�
	//�ö��ֵ�������ӵ��ļ����Ŀ��ֵ
	if( robot_control.smooth_Servo > p->Output ) 
	{
		robot_control.smooth_Servo -= robot_control.smooth_ServoStep;
		if( robot_control.smooth_Servo <= p->Output ) robot_control.smooth_Servo = p->Output;
	}
	else if( robot_control.smooth_Servo < p->Output )
	{
		robot_control.smooth_Servo += robot_control.smooth_ServoStep;
		if( robot_control.smooth_Servo >= p->Output  ) robot_control.smooth_Servo = p->Output;
	}
	else
	{
		robot_control.smooth_Servo = p->Output;
	}
	
	return (int)robot_control.smooth_Servo;
	#else
		//��ʹ���˲�ֱ�����
		return p->Output;
	#endif
}



/**************************************************************************
Function function: Determine the PWM value of the servo motor based on the position of the slide rail,
                   only available for top of the line models
Entrance parameters: servo encoder reading
Return value: Servo PWM
Author: WHEELTEC
�������ܣ�ͨ������λ���ж϶��PWMֵ,�����䳵�Ϳ���
��ڲ������������������
����  ֵ�����PWM
��    �ߣ�WHEELTEC
**************************************************************************/
short get_ServoPWM(short TmpPos)
{
	//��ʽ��ϰ汾,Ч����,��ÿ̨����ʽ��һ�����ͻ������������
//	//��������λ��������ֵ��ƫ��
//	//�����ʽ�붥��������Ҷ�������෴.
//	if( robot.type == 2 || robot.type == 3 )
//	{
//		//ͨ�������λ�ò²�����PWM��ֵ��С.��ʽ�ɲ����������ݽ�����ϵó�
//		return 1.57e+03 + 4.04e-01*TmpPos + (-5.82e-05)*pow(TmpPos,2);
//	}
//	else if( robot.type == 4 || robot.type == 5 )
//	{
//		//ͨ�������λ�ò²�����PWM��ֵ��С.��ʽ�ɲ����������ݽ�����ϵó�
////		return 1.46e+03 + (-4.05e-01)*TmpPos + (-9.42e-07)*pow(TmpPos,2);
//		return 1.48e+03 + (-2.39e-01)*TmpPos + (4.90e-06)*pow(TmpPos,2);	
//	}
//	else
//		return 0;
	
	uint16_t pwm_val = Akm_Servo.Mid;
	
	//����汾
	if( robot.type==4||robot.type==5 )
	{
			 if( TmpPos>=500&&TmpPos<=1000 )    pwm_val = Akm_Servo.Mid - 250;
		else if( TmpPos>1000 )                  pwm_val = Akm_Servo.Mid - 450;
		else if( TmpPos<=-500&&TmpPos>=-1000 )   pwm_val = Akm_Servo.Mid + 250;
		else if( TmpPos<-1000 )                   pwm_val = Akm_Servo.Mid + 450;
	}
	else if( robot.type==2 || robot.type==3 )
	{
			 if( TmpPos>=500&&TmpPos<=1000 )    pwm_val = Akm_Servo.Mid + 250;
		else if( TmpPos>1000 )                  pwm_val = Akm_Servo.Mid + 450;
		else if( TmpPos<=-500&&TmpPos>=-1000 )   pwm_val = Akm_Servo.Mid - 250;
		else if( TmpPos<-1000 )                   pwm_val = Akm_Servo.Mid - 450;
	}
	
	return pwm_val;
}

/**************************************************************************
Function Function: Servo Calibration Function. When the servo is in non self-locking mode and 
the position of the servo changes due to external factors, this function can be used for calibration
Entrance parameters: None
Return value: None
Author: SHEELTEC
�������ܣ����У������.������ڷ�����ģʽ��ʱ,��Ϊ������ص��¶��λ�÷����仯,ʹ�øú���У׼
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void ServoCalibration(void)
{
	short tmp_pwm;
	tmp_pwm = get_ServoPWM( robot.SERVO.Encoder ); //��ȡ��ǰλ�ö�ӦPWM��ֵ
	SERVO_TOP = tmp_pwm;                  //��ʼ�������λ��,�ñ���ֱ�Ӿ��������λ��
	PI_Controller_Reset(&PI_Servo);       //��λ�����PI������
	PI_Servo.Output = tmp_pwm;            //PI�����������ֵ,�����Ը���ֵ��Ϊ���������ٶ�ƽ��.
	robot_control.smooth_Servo = tmp_pwm; //ƽ��ֵ,����ʵ�ʵĿ�����.
}

//���������ģʽ���
static void Servo_UnLock_Check(uint8_t car_stopflag)
{
	static uint8_t now = 0 ;
	static uint8_t last = 0;
	
	if(SysVal.Time_count < CONTROL_DELAY ) return;
	
	//���������ģʽ���
	if ( robot.type>=2 && robot.type!=9 ) //�����䳵��
	{
		//С��ʹ�ܱ�־λ���
		now = car_stopflag;
		if( now==1 && last==0 ) //С����ʧ��
		{
			ServoState.UnLock_Pos = robot.SERVO.Encoder;//��¼ʧ��ʱ�����λ��
			ServoState.UnLock_Output = robot.SERVO.Output;//���PWMֵ
		}
		else if( now==0 && last==1 ) //С����ʹ��
		{
			if( fabs( ServoState.UnLock_Pos - robot.SERVO.Encoder ) > 100 ) //��ʧ���ڼ�,���λ�ñ����ı���,�����½���
			{
				ServoState.UnLock = 0;
				ServoCalibration();//�������λ��
				robot_control.ServoLow_flag = 1;//����������ģʽ,��֤���������λ(�����䰢����)
			}
		}
		last = now;
		
		//������ģʽ�˳����
		if( 1 == ServoState.UnLock )
		{
			//�ڷ������ڼ�,����Ƕȱ���Ӹı�,��Ҫ����У׼.
			if( fabs( robot.SERVO.Encoder - ServoState.UnLock_Pos ) > 100 ) ServoState.wait_Calib = 1;
			
			//����п�����ʱ,���������,������������ģʽ
			if( fabs( robot.SERVO.Target - ServoState.UnLock_Target ) > 50 || 1 == ValChangeCheck(BALANCE_TASK_RATE,Akm_Servo.Bias,3)) 
			{
				if( ServoState.wait_Calib ) 
				{
					ServoState.wait_Calib=0,ServoCalibration();//������Ҫ����У׼���
					if( fabs(robot_control.smooth_Vx) < 0.001f ) robot_control.ServoLow_flag = 1;//���С��������ʻ�У��ڽ���ʱʹ�õ��ٽ���
				}
				ServoState.UnLock = 0;//���������ģʽ
			}		
			
		} 
	}//�����ж�end
}
#endif
/**************************************************************************
Functionality: PI Controller Clearing Function - Clears the output residue of a PI (Proportional-Integral) 
               controller when the robot is disabled, aiming to reduce power consumption and motor noise.
Input Parameters: PI controller.
Return Value: None.
Author: WHEELTEC
�������ܣ�PI�������������,�ڻ����˽�ֹʱ���������࣬���͹����Լ����������
��ڲ�����PI������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static uint8_t PI_Clear_Output(PI_CONTROLLER* p)
{
	u8 state = 0;//�����־λ
	
	if( p->Output > 0 ) p->Output -- ;
	if( p->Output < 0 ) p->Output ++ ;
	
	//�ӽ�0ʱ,ֱ����0
	if( p->Output < 2.0f && p->Output > -2.0f ) p->Output = 0 , state = 1;
	
	//����������. 0:δ��� 1:������
	return state;
}

/*-------------------------------- Function for Inverse Kinematics Solution ------------------------------------*/
/*--------------------------------         �������˶�ѧ�����غ���          ------------------------------------*/
#if defined AKM_CAR

//���뻬��λ�ã��������ǰ����ǰ��ת��
float SteeringStructure_ForwardKinematic(short ori_pos)
{
    float pos;
    float tmp_theta;
    float tmp_C;

    pos = (float)ori_pos*0.0244f + 103;

    tmp_C = ( -1500.5f - pow(pos,2) ) / ( 140.4f*sqrt( pow(pos,2) + 3340.8f ) );

    tmp_theta = atan2(pos,57.8f) - asin( tmp_C ) - PI;

    return tmp_theta/PI * 180.0f + 72.39f;
}

//������ǰ��ת��,��������λ����Ŀ��ֵ
static int SteeringStructure_Reverse(float target_theta)
{
	float theta;
	float pos;
	
	//�����ʽ������
	if( 2 == robot.type || 3 == robot.type )
	{
		theta = target_theta - 1.26f;   //6768.4f
		pos = 70.2f * cos(theta) + sqrt( 6084.0f - pow((70.2f*sin(theta)+57.8f),2) ) - 103;
		pos /= 0.0244f;
		
		//��λ���������Ҫ��֤�ڿɴﷶΧ.
		pos = target_limit_float(pos,-2000,2000);
	}
	
	//�������������
	else if( 4 == robot.type || 5 == robot.type )
	{
		theta = target_theta - 1.170f;    //24831.4f
		pos = 95.15f * cos(theta) + sqrt( 24025.0f - pow((95.15f*sin(theta)+56.5f),2) ) - 191.54f;
		pos /= 0.0244f;
		
		//��λ���������Ҫ��֤�ڿɴﷶΧ.
		pos = target_limit_float(pos,-2000,2000);
	}

	return (int)pos;

}

/**************************************************************************
Function function: Ackermann inverse kinematics solution
Entrance parameters: X-axis velocity (in m/s), left front wheel angle (in rad)
Return value: None
Author: WHEELTEC
�������ܣ��������˶�ѧ���
��ڲ�����X����ٶ�(��λm/s)����ǰ�ֵ�ת��(��λrad)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void InverseKinematics_akm(float Vx,float Vz) 
{
	//�˶�ѧ����ǰ,���Ȳɼ�����ĵ�λ��У׼ֵ,�Ա��˶�ѧ�����������
	Akm_Servo.Bias = -get_DMA_ServoBias() / 5.0f;
	
	//�Ǳ궨�Ƴ���,����ݶ�������Ͳ�ͬ�����ƶ�Ӧ���˶�ѧ��������.
	if( robot.type==6 )
	{
	  
	}
	//SENIOR_AKM - ���䰢���� 
	else if(robot.type==0||robot.type==1||robot.type==9) 
	{
		//Ackerman car specific related variables //������С��ר����ر���
		float TurnR=0, Left_Angle=0;
		
		// For Ackerman small car, Vz represents the front wheel steering Angle
		//���ڰ�����С��Vz������ǰ��ת��Ƕ�
		Left_Angle = Vz;
		
		//��������ת��ĽǶ�
		const float limt_angle = 30.0f;
		//���޵�ת��뾶
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*robot.HardwareParam.WheelSpacing;
		//���ݼ���ת��뾶,��������תʱ������ֽǶ�
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*robot.HardwareParam.WheelSpacing));
		
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));
		
		//����ת��Ƕȼ������ǰ��ת��뾶
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*robot.HardwareParam.WheelSpacing;
		
		//����ת��뾶���־�,���������ֵ��ٶ�
		//Inverse kinematics //�˶�ѧ���
		if(Left_Angle!=0)
		{	//��������Ŀ���ٶ�,��λ��m/s
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //����ת��Ŀ���
		{	
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
		
		// The PWM value of the servo controls the steering Angle of the front wheel
		//���PWMֵ���������ǰ��ת��Ƕ�
		//ǰ��ת��������ֵ�Ķ�Ӧ��ϵ
//		robot.SERVO.Target    =  7.3f + 823.4f*Left_Angle + (-440.4f)*pow(Left_Angle,2) + (-28.8)*pow(Left_Angle,3);
		robot.SERVO.Target    =  7.85f + 847.0f*Left_Angle + (-394.0f)*pow(Left_Angle,2) + (-74.2f)*pow(Left_Angle,3);
		
		//���������� = ��ֵ + ��λ������ֵ + ����ֵ
		robot.SERVO.Output = Akm_Servo.Mid + Akm_Servo.Bias + robot.SERVO.Target;
	}
	
	//TOP_AKM_BS - ���䰢����С����ʽ����
	else if( robot.type==2 || robot.type==3 )
	{
		//ת��뾶�����ֽǶ�
		float TurnR = 0, Left_Angle = 0;
		
		//���ڰ�����С��Vz������ǰ��ת��Ƕ�
		Left_Angle = Vz;
		
		//��������ת��ĽǶ�
		const float limt_angle = 25.0f;
		
		//����Ƕ�ʱ�����˾���K
		const float K = 0.326f;
		
		//���޵�ת��뾶
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*K;
		
		//���ݼ���ת��뾶,��������תʱ������ֽǶ�
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*K));
		
		//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));//ע: ��+ ��-
		
		//����Ŀ��ֵ���㣺
		robot.SERVO.Target = SteeringStructure_Reverse(Left_Angle);
		//robot.SERVO.Target+=Akm_Servo.Bias;	//����ƫ��ֵ,ʵ��ǰ��ת�ǿ��õ�λ�����ڲ���
		
		//����ת��Ƕȼ������ǰ��ת��뾶
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*K;
		
		//������Ŀ��ֵ���㣺
		//����ת��뾶���־�,���������ֵ��ٶ�
		//Inverse kinematics //�˶�ѧ���
		if(Left_Angle!=0)
		{
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //����ת��Ŀ���
		{
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
		
	}
	
	else if ( robot.type==4 || robot.type==5 )
	{
		//ת��뾶�����ֽǶ�
		float TurnR = 0, Left_Angle = 0;
		
		//���ڰ�����С��Vz������ǰ��ת��Ƕ�
		Left_Angle = Vz;
		
		//��������ת��ĽǶ�
		const float limt_angle = 25.0f;
		
		//����Ƕ�ʱ�����˾���K
		const float K = 0.441f;
		
		//���޵�ת��뾶
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*K;
		
		//���ݼ���ת��뾶,��������תʱ������ֽǶ�
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*K));
		
		//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));//ע: ��+ ��-
		
		//����Ŀ��ֵ���㣺
		robot.SERVO.Target = SteeringStructure_Reverse(Left_Angle);
		
		//����ת��Ƕȼ������ǰ��ת��뾶
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*K;
		
		//������Ŀ��ֵ���㣺
		//����ת��뾶���־�,���������ֵ��ٶ�
		//Inverse kinematics //�˶�ѧ���
		if(Left_Angle!=0)
		{
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //����ת��Ŀ���
		{
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
	}
	else if( robot.type == 7 )
	{
		if( Vz > 0 ) robot.SERVO.Output = 1800;
		else if( Vz < 0 ) robot.SERVO.Output = 1100;
		else robot.SERVO.Output = 1500;
	}
}

/**************************************************************************
Function function: Convert the target forward velocity Vx and target angular velocity Vz sent by
                the upper computer into the left front wheel steering angle of the Ackermann car
Entrance parameters: target forward velocity Vx, target angular velocity Vz, unit: m/s, rad/s
Return value: Left front wheel steering angle of Ackermann car, unit: rad
�������ܣ�����λ��������Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz��ת��Ϊ������С������ǰ��ת��
��ڲ�����Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz����λ��m/s��rad/s
����  ֵ��������С������ǰ��ת�ǣ���λ��rad
**************************************************************************/
float Akm_Vz_to_Angle(float Vx,float Vz)
{
    float TurnR, Angle_Left;//ת��뾶,��ǰ�ֽǶ�

    if(Vz!=0 && Vx!=0)
    {
		//����ת��뾶
		TurnR = Vx/Vz;
		
        //ȷ��ת��뾶���ᳬ���˶�ѧ��ʽ�ļ��㷶Χ��
        if( TurnR > 0 && TurnR <= 0.5f*robot.HardwareParam.WheelSpacing ) TurnR = 0.5f*robot.HardwareParam.WheelSpacing+0.01f;
        if( TurnR < 0 && TurnR >= -0.5f*robot.HardwareParam.WheelSpacing ) TurnR = -0.5f*robot.HardwareParam.WheelSpacing-0.01f;
		
		//����ת��뾶ȷ����ǰ��ת�ǻ���
		Angle_Left=atan(robot.HardwareParam.AxleSpacing/(TurnR - 0.5f*robot.HardwareParam.WheelSpacing));

    }
    else
    {
        Angle_Left=0;
    }

    return Angle_Left;
}

/**************************************************************************
Function function: Differential robot inverse kinematics solution
Entrance parameters: X-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: WHEELTEC
�������ܣ����ٻ������˶�ѧ���
��ڲ�����X���ٶ�(��λm/s)��Z���ٶ�(��λ rad/s)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined DIFF_CAR
static void InverseKinematics_diff(float Vx,float Vz)
{
	robot.MOTOR_A.Target = Vx - Vz * robot.HardwareParam.WheelSpacing / 2.0f;
	robot.MOTOR_B.Target = Vx + Vz * robot.HardwareParam.WheelSpacing / 2.0f;
}

/**************************************************************************
Function function: inverse kinematics solution of the wheat wheel robot
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: WHEELTEC
�������ܣ����ֻ������˶�ѧ���
��ڲ�����X���ٶ�(��λm/s)��Y���ٶ�(m/s)��Z���ٶ�(rad/s)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined MEC_CAR
static void InverseKinematics_mec(float Vx,float Vy,float Vz)
{
    robot.MOTOR_A.Target = +Vy+Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = -Vy+Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = +Vy+Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = -Vy+Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}

/**************************************************************************
Function function: inverse kinematics solution of four-wheel drive robot
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author  :WHEELTEC
�������ܣ������������˶�ѧ���
��ڲ�����X���ٶ�(��λm/s)��Y���ٶ�(m/s)��Z���ٶ�(rad/s)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined _4WD_CAR
static void InverseKinematics_4wd(float Vx,float Vz)
{
    robot.MOTOR_A.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}

/**************************************************************************
Function function: Inverse kinematics solution for omnidirectional wheeled robots
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: WHEELTEC
�������ܣ�ȫ���ֻ������˶�ѧ���
��ڲ�����X���ٶ�(��λm/s)��Y���ٶ�(m/s)��Z���ٶ�(rad/s)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
#elif defined OMNI_CAR
static void InverseKinematics_omni(float Vx,float Vy,float Vz)
{
    robot.MOTOR_A.Target =  Vy + robot.HardwareParam.TurnRadiaus*Vz;
    robot.MOTOR_B.Target = -robot.HardwareParam.X_PARAMETER*Vx - robot.HardwareParam.Y_PARAMETER*Vy + robot.HardwareParam.TurnRadiaus*Vz;
    robot.MOTOR_C.Target = +robot.HardwareParam.X_PARAMETER*Vx - robot.HardwareParam.Y_PARAMETER*Vy + robot.HardwareParam.TurnRadiaus*Vz;
}

#endif

/*-------------------------------- Control class related functions ------------------------------------*/
/*--------------------------------        ��������غ���          ------------------------------------*/
//�����ƫϵ��
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //���־�ƫ,��Ӧ50~100��Ӧ1.0~1.2���ľ�ƫϵ��
	{
		if( diffparam>=50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //���־�ƫ,50~0��Ӧ1.0~1.2���ľ�ƫϵ��
	{
		if( diffparam<=50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//����������ʱ,Ĭ����1.
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ�ִ���˶�ѧ��⺯������������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�����ٶ�
����  ֵ����
**************************************************************************/
static void Drive_Motor(float T_Vx,float T_Vy,float T_Vz)
{
	// Function to Limit Maximum Target Speed Input 
	//�������Ŀ���ٶȽ������ֵ�޷�,��λm/s
	T_Vx = target_limit_float( T_Vx, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vy = target_limit_float( T_Vy, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vz = target_limit_float( T_Vz, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	//Smoothly control the target speed input.
	//�������Ŀ���ٶȽ���ƽ������
	if( charger.AllowRecharge==0 )
	{
		robot_control.smooth_Vx = Vel_SmoothControl(robot_control.smooth_Vx,T_Vx,robot_control.smooth_MotorStep);
		robot_control.smooth_Vy = Vel_SmoothControl(robot_control.smooth_Vy,T_Vy,robot_control.smooth_MotorStep);
		
		//����������,Vz������ǰ�ֽǶ�.���ﲻ��ƽ������
		#if defined AKM_CAR
			robot_control.smooth_Vz = T_Vz;
		#else
			robot_control.smooth_Vz = Vel_SmoothControl(robot_control.smooth_Vz,T_Vz,robot_control.smooth_MotorStep);
		#endif	
	}
	else
	{
		robot_control.smooth_Vx = T_Vx;//�Զ��س�ģʽ�²�ʹ��ƽ������,���С������Ӧ�ٶȶԽ�.
		robot_control.smooth_Vy = T_Vy;
		robot_control.smooth_Vz = T_Vz;
	}

	// Call Inverse Kinematics Function for Corresponding Vehicle Model to Obtain Target Values for Wheels and Steering Angle
	//���ö�Ӧ���͵��˶�ѧ��⺯��,��ø������ӵ�Ŀ��ֵ������Ƕ�
	#if defined AKM_CAR
		InverseKinematics_akm(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif defined DIFF_CAR
		InverseKinematics_diff(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif defined MEC_CAR
		InverseKinematics_mec(robot_control.smooth_Vx,robot_control.smooth_Vy,robot_control.smooth_Vz);
	#elif defined _4WD_CAR
		InverseKinematics_4wd(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif OMNI_CAR
		InverseKinematics_omni(robot_control.smooth_Vx,robot_control.smooth_Vy,robot_control.smooth_Vz);
	#endif

	//��ƫϵ������
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
	// Limit Final Wheel Speeds or Servo Travel After Kinematic Analysis.
	//�����˶�ѧ������������������ٶ������Ƕ�,��������յ����ٻ����г̽����޷�.
	robot.MOTOR_A.Target = target_limit_float( robot.MOTOR_A.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_B.Target = target_limit_float( robot.MOTOR_B.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	#if defined AKM_CAR || defined DIFF_CAR
		robot.MOTOR_A.Target*=LeftWheelDiff;
		robot.MOTOR_B.Target*=RightWheelDiff;
	#elif defined MEC_CAR || defined _4WD_CAR
		robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
		robot.MOTOR_D.Target = target_limit_float( robot.MOTOR_D.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );	
		robot.MOTOR_A.Target*=LeftWheelDiff;
		robot.MOTOR_B.Target*=LeftWheelDiff;
		robot.MOTOR_C.Target*=RightWheelDiff;
		robot.MOTOR_D.Target*=RightWheelDiff;
	#elif defined OMNI_CAR
		robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );	
		robot.MOTOR_B.Target*=LeftWheelDiff;
		robot.MOTOR_C.Target*=RightWheelDiff;
	#endif
}

/**************************************************************************
Function function: Control the corresponding function. After inverse kinematics solution, 
obtain the target values of each executing mechanism of the robot, and control the motion 
of the executing mechanism through PID control
Entrance parameters: None
Return value: None
Author: WHEELTEC
�������ܣ�������Ӧ����.�����˶�ѧ����ó������˸���ִ�л�����Ŀ��ֵ,ͨ��PID����ִ�л����˶�
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void ResponseControl(void)
{
	#if defined AKM_CAR
	//PI����
	// --- ���������������Ӳ������߼� ---
	// �ж��Ƿ��ھ��Ե�ֱ����ʻָ�������Ŀ���ٶȲ�ֵ��С���������˶��У�
	if(fabsf(robot.MOTOR_A.Target - robot.MOTOR_B.Target) < 0.001f && fabsf(robot.MOTOR_A.Target) > 0.01f) 
	{
		// 1. �������ֵı���������ȡƽ������Ϊ��������ĵ�ǰ�ٶȷ���
		float avg_encoder = (robot.MOTOR_A.Encoder + robot.MOTOR_B.Encoder) / 2.0f;
		
		// 2. ��ʹ�� Motor A �� PI �����������ͳһ�� PWM ���
		int sync_output = Incremental_MOTOR(&PI_MotorA, avg_encoder, robot.MOTOR_A.Target);
		
		// 3. ǿ��������ʹ����ȫ��ͬ�� PWM ��������е��ֹЧ������ֹ����������
		robot.MOTOR_A.Output = sync_output;
		robot.MOTOR_B.Output = sync_output;
		
		// 4. �ϸ񱣳� Motor B �� PI ״̬�� A ͬ������ֹ����һ��ת�򣨽����ֹ��ʱ��������ͻ��
		PI_MotorB.Bias = PI_MotorA.Bias;
		PI_MotorB.LastBias = PI_MotorA.LastBias;
		PI_MotorB.LastestBias = PI_MotorA.LastestBias; 
		PI_MotorB.Output = sync_output;
	}
	else 
	{
		// ԭ���߼�����ֱ����ʻ������ͣ��״̬�����Զ������� PI���������ٹ����ɲ����
		if(fabsf(robot.MOTOR_A.Target) < 0.01f) {PI_Controller_Reset(&PI_MotorA); robot.MOTOR_A.Output = 0;}
		else robot.MOTOR_A.Output = Incremental_MOTOR( &PI_MotorA , robot.MOTOR_A.Encoder , robot.MOTOR_A.Target );
		
		if(fabsf(robot.MOTOR_B.Target) < 0.01f) {PI_Controller_Reset(&PI_MotorB); robot.MOTOR_B.Output = 0;}
		else robot.MOTOR_B.Output = Incremental_MOTOR( &PI_MotorB , robot.MOTOR_B.Encoder , robot.MOTOR_B.Target );
	}
	
	//The servo of the top of the line Ackermann model only requires PI control. \
	The high-end model does not come with steering rail feedback
	//�����䰢�������͵Ķ����ҪPI����.���䳵�Ͳ���ת�򻬹췴��.
	if( robot.type>=2 && robot.type!=9 && robot.type!=7 )
	{
		if(ServoState.UnLock==0)
			robot.SERVO.Output   = Incremental_Servo( &PI_Servo  , robot.SERVO.Encoder  ,  robot.SERVO.Target + Akm_Servo.Bias );	
	}
//	robot.MOTOR_A.Output = 2000;
//	robot.MOTOR_B.Output = 2000;
	//����������ִ�л���
	      if( robot.type == 6 )                      Set_Pwm(0,0,0,0,0); //�Ǳ�Ԥ�����Ƴ�,����ݵ���ͺ��������ֵ����������
	else if ( robot.type <= 1 || robot.type == 9)   Set_Pwm( robot.MOTOR_A.Output , robot.MOTOR_B.Output , 0 , 0 ,robot.SERVO.Output ); //���䰢����-MD36���
	else if ( robot.type >= 2 && robot.type <= 5 )  Set_Pwm(-robot.MOTOR_A.Output ,-robot.MOTOR_B.Output , 0 , 0 ,robot.SERVO.Output ); //���䰢����-MD60���
	else if ( robot.type == 7 )                     Set_Pwm( 0 , 0 , 0 , 0 , robot.SERVO.Output ); //��װ����ר��.�����������
	else if ( robot.type == 8 )                     Set_Pwm( 0 , 0 , 0 , 0 , 1500 ); //��װ����ר��.�ö���������м�λ��У׼
	
	#elif defined DIFF_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	
	if( robot.type<=1 )  Set_Pwm( robot.MOTOR_A.Output , robot.MOTOR_B.Output ,0,0,0);//�������-MD36���
	else                 Set_Pwm(-robot.MOTOR_A.Output ,-robot.MOTOR_B.Output ,0,0,0);//�������-MD60���
	
	#elif defined MEC_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	robot.MOTOR_D.Output = Incremental_MOTOR(&PI_MotorD,robot.MOTOR_D.Encoder,robot.MOTOR_D.Target);
	
	if( robot.type<=2 ) Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output,0);//��������-MD36���
	else                Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output,-robot.MOTOR_C.Output,-robot.MOTOR_D.Output,0);//������콢����-MD60���
	
	#elif defined _4WD_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	robot.MOTOR_D.Output = Incremental_MOTOR(&PI_MotorD,robot.MOTOR_D.Encoder,robot.MOTOR_D.Target);
	
	if( robot.type<=3 ) Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output,0);//��������-MD36���
	else                Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output,-robot.MOTOR_C.Output,-robot.MOTOR_D.Output,0);//������콢����-MD60���
	
	#elif defined OMNI_CAR
	 
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	
	if( robot.type<=1 ) Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output, robot.MOTOR_C.Output,0,0);//����ȫ����-MD36���
	else                Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output,-robot.MOTOR_C.Output,0,0);//����ȫ����-MD60���

	#endif
}

/**************************************************************************
Functionality: Non-Control Response Function - Executes when the robot encounters an error, prohibiting the robot's movement.
Input Parameters: Set the robot's stop mode: 0: Set to unclamp axes, allowing the robot to be freely pushed. 
                  1: Set to clamp axes, locking all actuators.
Return Value: None
Author: WHEELTEC
�������ܣ��ǿ�����Ӧ����,�������˴��ڱ���ʱ,��ִ�д˺���.��ֹ�����˵��˶�
��ڲ��������û�����ֹͣ�˶��ķ�ʽ: 0:����Ϊ����,�����˿��������ƶ�  1:����Ϊ����,��������ס����ִ�л���
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void UnResponseControl(uint8_t mode)
{
	//������Ч��У��
	if(mode!=UN_LOCK && mode!=LOCK) mode = UN_LOCK;
	
	//����ʧ��
	if(mode==UN_LOCK)
	{
		//��λPI������,��������
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
		
		//����0ֹͣ�����ת��
		Set_Pwm( 0 , 0 , 0 , 0 , 0 );
	}
	
	//����ʧ��,�޷��ƶ�С��
	else if( mode==LOCK )
	{
		Drive_Motor(0,0,0);
		ResponseControl();
	}
}

/**************************************************************************
Functionality: Send Control Values to Actuators - Distributes control values to each actuator (A, B, C, D wheels and servo).
Input Parameters: Control value for A wheel motor, control value for B wheel motor, control value for C wheel motor, 
                   control value for D wheel motor, control value for servo.
Return Value: None.
Author: WHEELTEC
�������ܣ��·�������������ִ�л���
��ڲ�����A�ֵ����������B�ֵ����������C�ֵ����������D�ֵ�������������������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)
{
	#if defined AKM_CAR || defined DIFF_CAR
	
		//�������
		if( m_a < 0 ) AIN1=1,AIN2=0;
		else          AIN1=0,AIN2=1;
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=1,V1_0_BIN2=0;
			else          V1_0_BIN1=0,V1_0_BIN2=1;
		}
		else if( SysVal.HardWare_Ver==V1_1 )
		{
			if( m_b > 0 ) BIN1=1,BIN2=0;
			else          BIN1=0,BIN2=1;
		}
		
		PWMA = abs(m_a);
		PWMB = abs(m_b);
	
		#if defined AKM_CAR
		//�������
		if( robot.type == 0 || robot.type == 1 || robot.type==9 )//���䰢��������
		{
			SERVO_SENIOR = servo;
		}
		else //���䰢��������
		{
			SERVO_TOP = servo;
		}
		#endif
		
	#else 
		
		//�������
		if( m_a > 0 ) AIN1=0,AIN2=1;
		else          AIN1=1,AIN2=0;
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=0,V1_0_BIN2=1;
			else          V1_0_BIN1=1,V1_0_BIN2=0;
		}
		else if( SysVal.HardWare_Ver==V1_1 )
		{
			if( m_b > 0 ) BIN1=0,BIN2=1;
			else          BIN1=1,BIN2=0;
		}
	
		if( m_c > 0 ) CIN1=1,CIN2=0;
		else          CIN1=0,CIN2=1;
		
		if( m_d > 0 ) DIN1=1,DIN2=0;
		else          DIN1=0,DIN2=1;
		
		PWMA = abs(m_a);
		PWMB = abs(m_b);
		PWMC = abs(m_c);
		PWMD = abs(m_d);
	#endif
}

/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬������ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ��������ƣ�1����������0����
**************************************************************************/
static u8 Turn_Off(void)
{
    u8 temp = 0;
	static uint8_t saveflag = 0;
	
	//��ѹ������Ҫ�󣺵�ѹ����20V,�Ҳ����Զ��س�״̬.(���ǽ����Զ��س�״̬,����Ե�ѹ)
	if( robot.voltage < 20.0f && charger.AllowRecharge==0 ) temp = 1;

	//��ͣ���ر�����
	if( EN==0 ) temp = 1;
	
	//�������Լ����
	if( robot_check.errorflag==1 ) temp = 1;

	//������ͨ�����������˼�ͣ
	if( robot_control.SoftWare_Stop==1 ) temp = 1;
	
	//С����ʧ�ܱ�Ϊʹ�ܵĹ���,���Ŀ���ٶ�.
	if(temp==0 && saveflag==1)
	{
		robot_control.Vx = 0;
		robot_control.Vy = 0;
		robot_control.Vz = 0;
		robot_control.smooth_Vx = 0;
		robot_control.smooth_Vy = 0;
		robot_control.smooth_Vz = 0;
	}
	saveflag = temp;
	
	return temp;
}

/**************************************************************************
Function: Processes the command sent by APP through usart x
Input   : none
Output  : none
�������ܣ���APPͨ������x���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
static void Get_APPcmd(void)
{
	/*  APPҡ�����ٶȷ���
	     A
	     ^  +Vx       ^ +Vz
   H     |       B    ��
	     |
   G----------> -Vy  C
	     |
   F     |    D
	     E
	*/
	
	short m_sign = 1;         //���ű���
	float base_vz= PI/4.0f;   //Z���ٶȻ�׼
	
	//��ȫ���ƶ�С��
	#if defined AKM_CAR || defined DIFF_CAR || defined _4WD_CAR
	
		#if defined AKM_CAR
			m_sign = -1;                  //������Vz��ʾǰ��ת��,���ַ��������Ҫ�޸�.
			base_vz = angle_to_rad(30);
		#endif
	
	switch( appkey.DirectionFlag ) 
	{
		case 1: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = 0               ; break; //A
		case 2: robot_control.Vx = robot_control.rc_speed , robot_control.Vz =-base_vz         ; break; //B
		case 3: robot_control.Vx = 0 ,                      robot_control.Vz =-base_vz         ; break; //C
		case 4: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = (base_vz)*m_sign; break; //D
		case 5: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = 0               ; break; //E
		case 6: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz =-(base_vz)*m_sign; break; //F
		case 7: robot_control.Vx = 0 ,                      robot_control.Vz = base_vz         ; break; //G
		case 8: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = base_vz         ; break; //H
		
		default : robot_control.Vx=0;robot_control.Vy=0;robot_control.Vz=0; break;
	}
	
	
	//��ȫ���ƶ�С��
	#elif defined MEC_CAR || defined OMNI_CAR
	
	switch( appkey.DirectionFlag ) 
	{
		case 1: robot_control.Vx = robot_control.rc_speed , robot_control.Vy = 0                      ; break; //A
		case 2: robot_control.Vx = robot_control.rc_speed , robot_control.Vy =-robot_control.rc_speed ; break; //B
		case 3: robot_control.Vx = 0 ,                      robot_control.Vy =-robot_control.rc_speed ; break; //C
		case 4: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy =-robot_control.rc_speed ; break; //D
		case 5: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy = 0                      ; break; //E
		case 6: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy = robot_control.rc_speed ; break; //F
		case 7: robot_control.Vx = 0 ,                      robot_control.Vy = robot_control.rc_speed ; break; //G
		case 8: robot_control.Vx = robot_control.rc_speed , robot_control.Vy = robot_control.rc_speed ; break; //H
		
		default : robot_control.Vx=0;robot_control.Vy=0;robot_control.Vz=0 , m_sign=0 ; break;
	}
	
	//С��������x��y������ʱ,���Z������
	if( m_sign==0 )
	{
		     if( appkey.TurnFlag==1 ) robot_control.Vz =  base_vz;
		else if( appkey.TurnFlag==2 ) robot_control.Vz = -base_vz;
		else                          robot_control.Vz = 0;
	}
	
	#endif

	//��λת�� mm/s -> m/s
	robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;
	robot_control.Vz = robot_control.Vz * ( robot_control.rc_speed/500.0f );//Z���ٶȸ���ң���ٶȵ���.

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
static void Remote_Control(void)
{
    //Data within 1 second after entering the model control mode will not be processed
    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=BALANCE_TASK_RATE;
    int Yuzhi=100; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

    //limiter //�޷�
    int LX,LY,RY,RX,Remote_RCvelocity;
	float base_vz = PI/4.0f ;//Z���ٶȻ�׼
	
	//������Z�������ǰ�ֽǶ�,���û�׼ֵΪ 30 ��
	#if defined AKM_CAR
		base_vz = angle_to_rad(30.0f);
	#endif
	
	//4��ͨ��ԭʼֵ�޷�
	remoter.ch1 = target_limit_int(remoter.ch1,1000,2000);
	remoter.ch2 = target_limit_int(remoter.ch2,1000,2000);
	remoter.ch3 = target_limit_int(remoter.ch3,1000,2000);
	remoter.ch4 = target_limit_int(remoter.ch4,1000,2000);

    //Front and back direction of left rocker. Control forward and backward.
    //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=remoter.ch2-1500;
    
	//The left joystick's horizontal directions control lateral \
	  movement for omnidirectional mobile vehicles.
    //��ҡ�����ҷ���,����ȫ���ƶ�С���ɿ������Һ���
    LY=remoter.ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //��ҡ��ǰ��������/�Ӽ��١�
    RX=remoter.ch3-1500;
	
    //Right stick left and right. To control the rotation.
    //��ҡ�����ҷ��򡣿�����ת��
    RY=-(remoter.ch1-1500);//��ת

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	//���ŵ����ٶ� Remote_RCvelocityȡֵ:0~1000
    Remote_RCvelocity= robot_control.rc_speed + RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //�Ժ�ģң�ؿ���������д���
	
	robot_control.Vx = LX * (float)Remote_RCvelocity/500.0f;
	robot_control.Vy =-LY * (float)Remote_RCvelocity/500.0f;
	
	//��׼:base_vz,��������:RY/500,ȡֵ[-1~1],���ű���:(Remote_RCvelocity/500.0f),ȡֵ[0,2]
	robot_control.Vz = base_vz*((float)RY/500.0f) * ((float)Remote_RCvelocity/500.0f) ; 

	//�ǰ���������,�ں���ʱת����ȡ��.����ҡ�˲����߼�.
	#if !defined AKM_CAR
	if(  robot_control.Vx < 0 ) robot_control.Vz = -robot_control.Vz;
	#endif

    //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s
	robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;

    //Data within 1 second after entering the model control mode will not be processed
    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) robot_control.Vx=0,robot_control.Vy=0,robot_control.Vz=0,thrice--;

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
//xbox360��Ϸ�ֱ������ص�����
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//����start����
	if( keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	if( gamepad_brand == Xbox360 )
	{
		//�ֱ��Ӽ���
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed -= 50;
		else if( keyid == Xbox360KEY_RB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed += 50;
		
		if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
	}
	else if(  gamepad_brand == PS2_USB_Wiredless )
	{
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed += 50;
		else if( keyid == Xbox360_PaddingBit && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK ) )
			robot_control.rc_speed -= 50;
		if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
	}
	
	
	//�𶯼�����ȡ��
	if( keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK )
	{
		if( GamePadInterface->Vib_EN )
		{
			GamePadInterface->SetVibration(0,127);
			vTaskDelay(50);
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
		}
		else
		{
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
			vTaskDelay(50);
			GamePadInterface->SetVibration(0,127);
		}	
	}
}

//����USB�ֱ��ص�����
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//����start����
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//�ֱ��Ӽ���
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed += 50;
	
	if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
}

//����PS2�ֱ��ص�����,��USB��
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//����start����
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//�ֱ��Ӽ���
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed += 50;
	
	if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
}


//�ֱ���ӳ�亯��
static uint8_t map_to_vib(float x) {
    // ������뷶Χ�������� [0.2, 1.2] ��
    if (x < 0.1f) return 0;
    if (x > 1.2f) x = 1.2f;

    // ����ӳ��
    float result = 255.0f * (x - 0.1f) / 1.1f;

    // �������벢ת��Ϊ uint8_t
    return (uint8_t)(result + 0.5f);
}

static void PS2_control(void)
{
	float LX=127,LY=127,RX=127;
	float ThrottleTri = 255;
	float base_vz = PI/4.0f ;//Z���ٶȻ�׼
	
	//ǰ��ҡ��
	LY = GamePadInterface->LY - 127;
	
	//���Һ���
	LX = 127 - GamePadInterface->LX;
	
	//˳��ʱ��
	RX = 127 - GamePadInterface->RX;
	
	//ҡ��΢С���ȹ���
	if( fabs(LY)<20 ) LY = 0;
	if( fabs(LX)<20 ) LX = 0;
	if( fabs(RX)<20 ) RX = 0;
	
	//���xbox360�ֱ������Ϊģ����ʱ������ʹ�ð������
	if( gamepad_brand == Xbox360 )
	{
		//ǰ��ҡ����ֵʱ,���ð����ֵ
		if( (int)LY == 0 )
		{
			if( GamePadInterface->LT == 0 && GamePadInterface->RT != 0 )
				ThrottleTri =  GamePadInterface->RT, LY = 127;
			else if( GamePadInterface->LT != 0 && GamePadInterface->RT == 0 )
				ThrottleTri =  -GamePadInterface->LT,LY = 127;
			else
				ThrottleTri = 0;
		}
	}
	
	//���usb�����ֱ�,�ڷ�ģ����ģʽ�µ�ҡ��ֵӳ��
	else if( gamepad_brand == PS2_USB_Wired ||  gamepad_brand == PS2_USB_WiredV2 )
	{
		if( fabs(RX)<0.0001f )
		{
			if( GamePadInterface->getKeyState(PS2KEY_4PINK) )
				RX = 127;
			else if( GamePadInterface->getKeyState(PS2KEY_2RED) )
				RX = -127;
		}
	}
	
	robot_control.Vx = (LY/127.0f) * robot_control.rc_speed * (ThrottleTri/255.0f);
	robot_control.Vy = (LX/127.0f) * robot_control.rc_speed;
	robot_control.Vz = base_vz * (RX/127.0f) * ( robot_control.rc_speed/500.0f );
	
	#if !defined AKM_CAR
		if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;
	#endif

    //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s
    robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
	
	//���ݼ��ٶȷ�Ӧ��������ﵽ�ֱ�
	#include "bsp_gamepad.h"
	
	//Z�������ж���ǿ��
	float now_z = imu.accel.z/1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);
	
	#if defined MEC_CAR || defined OMNI_CAR
	const float vib_strength = 0.6f;
	#else
	const float vib_strength = 0.1f;
	#endif
	
	//��ӳ�䵽�ֱ�
	if( strength>vib_strength && SysVal.Time_count>CONTROL_DELAY)
	{
		if( GamePadInterface->SetVibration!=NULL )
			GamePadInterface->SetVibration(map_to_vib(strength),0);
	}
	last_z = now_z;
		
//	//TODO:����PS2����
//    float LX,LY,RY;
//    int Yuzhi=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
//	
//	float base_vz = PI/4.0f ;//Z���ٶȻ�׼
//	
//	//��� usb����ps2�ֱ� ����ģʽ����ת��
//	if( SysVal.HardWare_Ver==V1_1 )
//	{
//		if( ps2_type == Normal_PS2 )
//		{
//			if( ps2_val.RX==128 && ps2_val.RY==128 )
//			{
//				if( Read_PS2_KEY(R_4PINK )==PS2_KEY_ON )  ps2_val.RX  = 0;
//				if( Read_PS2_KEY(R_2RED  )==PS2_KEY_ON )  ps2_val.RX = 255;
//				if( Read_PS2_KEY(R_1GREEN)==PS2_KEY_ON )  ps2_val.RY = 0;
//				if( Read_PS2_KEY(R_3BLUE )==PS2_KEY_ON )  ps2_val.RY = 255;
// 			}
//		}
//	}
//	
//    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
//    //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
//    LY=-(ps2_val.LX-128);//ȡֵ[-128,128]
//    LX=-(ps2_val.LY-128);
//    RY=-(ps2_val.RX-128);

//    //Ignore small movements of the joystick //����ҡ��С���ȶ���
//    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
//    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
//    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
//	
//	static uint8_t low_fre = 0;
//	if(++low_fre==10)//���Ͱ���ɨ��Ƶ��
//	{
//		low_fre = 0;
//		//PS2�ֱ�,����1�ſɼ���,2�ſɼ���
//			 if( Read_PS2_KEY(L1_KEY)==PS2_KEY_ON ) robot_control.rc_speed+=20;
//		else if( Read_PS2_KEY(L2_KEY)==PS2_KEY_ON ) robot_control.rc_speed-=20;
//		
//		//�ٶ������Сֵ����
//		robot_control.rc_speed = target_limit_float(robot_control.rc_speed,0,robot_control.limt_max_speed*1000);
//	}

//	#if defined AKM_CAR
//		base_vz = angle_to_rad(30.0f);//������Z���׼�Ƕ�
//	#endif
//	
//    //Handle PS2 controller control commands
//    //��PS2�ֱ�����������д���
//	robot_control.Vx = (LX/128.0f) * robot_control.rc_speed;
//	robot_control.Vy = (LY/128.0f) * robot_control.rc_speed;
//	robot_control.Vz = base_vz * (RY/128.0f) * ( robot_control.rc_speed/500.0f );
//	
//	#if !defined AKM_CAR
//		if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;
//	#endif

//    //Unit conversion, mm/s -> m/s
//    //��λת����mm/s -> m/s
//    robot_control.Vx = robot_control.Vx/1000.0f;
//	robot_control.Vy = robot_control.Vy/1000.0f;

//    //Control target value is obtained and kinematics analysis is performed
//    //�õ�����Ŀ��ֵ�������˶�ѧ����
//    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Functionality: Robot Self-Check Function - Performs a self-check on the robot to verify if there are any errors in the wiring of motors, encoders, and drivers.
Input Parameters: None.
Return Value: None.
Author: WHEELTEC
�������ܣ� �������Լ캯��,�������˵�������������������Ƿ���ڴ������
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void robot_mode_check(void)
{
	//TODO:�Լ����,ͬ�����ӷ����ж�.
	
	#define OVERLOAD_PWM 8000 //����PWMԤ��ֵ
	#define ENCODER_LOST_PWM 5500 //������δ��ʱ,PWM������ֵ
	
    static u8 init=0;//��ʼ��
    if( 0 == init ) 
	{
		ROBOT_SELFCHECK_t_Init(&robot_check);
		init=1;
	}

    if( EN==1 && robot_check.errorflag ==0 ) //��������ʹ�ü�ͣ���������Լ�Ĺ���
    {
		#if defined OMNI_CAR //ȫ����С���ṹ����,�����Լ��߼�
			
			//1��ȫ�����Լ�ʱ,��B��C���ת��.��A·������ֵ��˵����������������߽Ӵ�(/4�����ķ�һȦ)
			if( robot_check.check_a > robot.HardwareParam.Encoder_precision/4 || \
				robot_check.check_a < -robot.HardwareParam.Encoder_precision/4	)
			{
				Buzzer_AddTask(1,20);
				robot_check.errorflag = 1;
			}
			
			//2�����Ͳ��Ի���߲��Ե��µĵ���������
			if( robot_check.check_b > robot.HardwareParam.Encoder_precision/4 || \
				robot_check.check_c < -robot.HardwareParam.Encoder_precision/4 )
			{
				Buzzer_AddTask(2,20);
				robot_check.errorflag = 1;
			}
			
			//3��������PWM���������ֵ,��Ӧ�ı��������ݺ�С(����ת������12��)��������,����Ϊ��������δ��
			if( ( fabs( robot.MOTOR_B.Output ) > ENCODER_LOST_PWM && robot_check.check_b > -robot.HardwareParam.Encoder_precision/30 ) || \
				( fabs( robot.MOTOR_C.Output ) > ENCODER_LOST_PWM && robot_check.check_c < robot.HardwareParam.Encoder_precision/30 ) )
			{
				Buzzer_AddTask(3,20);
				robot_check.errorflag = 1;
			}
			
			//4��������PWM�������������ֵ,����ΪС������
			if( fabs( robot.MOTOR_A.Output ) > OVERLOAD_PWM || fabs( robot.MOTOR_B.Output ) > OVERLOAD_PWM || \
				fabs( robot.MOTOR_C.Output ) > OVERLOAD_PWM )
			{
				Buzzer_AddTask(4,20);
				robot_check.errorflag = 1;
			}
			
		#else
			static int last_a=0,last_b=0,last_c=0,last_d=0;
			static u8 err_time = 0;
			
			//1�����⳵�ַ�ת 1/4 Ȧ ��Ϊ�Ǳ������߻������߽ӷ�,�����ǳ���ѡ��.
			if( robot_check.check_a < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_b < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_c < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_d < -robot.HardwareParam.Encoder_precision/4  ) 
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(1,20);
			}
			
			//2��������PWM���������ֵ,��Ӧ�ı��������ݺ�С(����ת������12��)��������,����Ϊ��������δ��
			if( ( abs( robot.MOTOR_A.Output ) > ENCODER_LOST_PWM && robot_check.check_a < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_B.Output ) > ENCODER_LOST_PWM && robot_check.check_b < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_C.Output ) > ENCODER_LOST_PWM && robot_check.check_c < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_D.Output ) > ENCODER_LOST_PWM && robot_check.check_d < robot.HardwareParam.Encoder_precision/30 )  )
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(2,20);
			}
			
			//3��������PWM�������������ֵ,����ΪС������
			if( abs( robot.MOTOR_A.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_B.Output ) > OVERLOAD_PWM || \
				abs( robot.MOTOR_C.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_D.Output ) > OVERLOAD_PWM	)
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(3,20);
			}
			
			//4�����ֶ���ۼ�ֵ����,����1�е��쳣,Ҳ�����Ƕ��ִ����ۼƵ��ӵı���.
			if( abs(robot_check.check_a) < abs(last_a) - 50 || abs(robot_check.check_b) < abs(last_b) - 50 || abs(robot_check.check_c) < abs(last_c) - 50|| abs(robot_check.check_d) < abs(last_d) - 50 )
			{
				err_time++;
				if( err_time > 20 )  robot_check.errorflag = 1 , Buzzer_AddTask(4,20);	
			}
		
			last_a = robot_check.check_a;
			last_b = robot_check.check_b;
			last_c = robot_check.check_c;
			last_d = robot_check.check_d;
		#endif
    }
}

/**************************************************************************
Functionality: Read Feedback Values - Retrieves feedback values from each actuator of the robot.
Input Parameters: None.
Return Value: None.
Author: WHEELTEC
�������ܣ���ȡ��������ִ�л����ķ���ֵ
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void Get_Robot_FeedBack(void)
{
    //Retrieves the original data of the encoder
    //��ȡ��������ԭʼ����
    short Encoder_A_pr,Encoder_B_pr;
	
	//������ԭʼֵ,��ȡ�����.
    Encoder_A_pr = Read_Encoder(Encoder_A);
    Encoder_B_pr = Read_Encoder(Encoder_B);
	
	#if !defined AKM_CAR && !defined DIFF_CAR
		short Encoder_C_pr;
		Encoder_C_pr = Read_Encoder(Encoder_C);
	#endif
	
	#if defined MEC_CAR || defined _4WD_CAR
		short Encoder_D_pr;
		Encoder_D_pr = Read_Encoder(Encoder_D);
	#endif
	
	//�����ƫϵ��
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
    //The encoder converts the raw data to wheel speed in m/s
    //������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
	#if defined AKM_CAR
	
		//δ����Լ�ʱ�ɼ����������ݽ����ж�
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
		}
		
		//����������ı�������ԭʼ����ת��Ϊ m/s 
		//T法: 超时100ms则归零速度 / T-method: zero velocity on 100ms timeout
		uint32_t now_us = get_us_tick();
		if ((now_us - last_pulse_update[0]) > 100000u) encoder_T_velocity_raw[0] = 0.0f;
		if ((now_us - last_pulse_update[1]) > 100000u) encoder_T_velocity_raw[1] = 0.0f;
		//T法速度已由EXTI ISR计算, 此处除以轮径补偿系数并经Kalman滤波
//		any_printf(USART1, "%.4f,%.4f\r\n",encoder_T_velocity_raw[0], encoder_T_velocity_raw[1]);
		//T-method velocity is computed in EXTI ISR; apply wheel correction and Kalman filter
		robot.MOTOR_A.Encoder = Kalman_Filter_1D(&Kalman_MotorA, encoder_T_velocity_raw[0] / LeftWheelDiff);
		robot.MOTOR_B.Encoder = Kalman_Filter_1D(&Kalman_MotorB, encoder_T_velocity_raw[1] / RightWheelDiff);
		any_printf(USART1, "%.4f,%.4f\r\n",robot.MOTOR_A.Encoder, robot.MOTOR_B.Encoder);
        //Gets the position of the slide, representing the front wheel rotation Angle
        //��ȡ����λ��,����ǰ��ת�ǽǶ�.����ֵ��ADC2����ɼ�,DMA��ɰ���,�˴����账�����ݼ���
		robot.SERVO.Encoder = get_DMA_SlideRes();
		robot.SERVO.Encoder = Slide_Mean_Filter(robot.SERVO.Encoder);//���ɼ��������ƽ���˲�
	
	#elif defined DIFF_CAR
		//δ����Լ�ʱ�ɼ����������ݽ����ж�
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
		}
		
		robot.MOTOR_A.Encoder =   Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#elif defined MEC_CAR || defined _4WD_CAR	
		//δ����Լ�ʱ�ɼ����������ݽ����ж�
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b +=  Encoder_B_pr;
			robot_check.check_c += -Encoder_C_pr;
			robot_check.check_d += -Encoder_D_pr;
		}
		robot.MOTOR_A.Encoder =   Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_B.Encoder =   Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
		robot.MOTOR_D.Encoder = - Encoder_D_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#elif defined OMNI_CAR	
		//δ����Լ�ʱ�ɼ����������ݽ����ж�
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a += -Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
			robot_check.check_c += -Encoder_C_pr;
		}
		
		robot.MOTOR_A.Encoder = - Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ;
		robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#endif
}

/*-------------------------------- Robot state detection related functions ------------------------------------*/
/*--------------------------------         ������״̬�����غ���          ------------------------------------*/
/**************************************************************************
Function Description: The function clears residual motor control signals when the robot \
  remains stationary on a non-sloping surface for a certain period of time. This helps  \
  reduce power consumption and motor noise of the robot.
Input Parameters: None
Return Value: None
Author: WHEELTEC
�������ܣ�С���ڷ�б���Ͼ�ֹ����һ��ʱ��,����������Ŀ�����.���ͻ����˹����Լ����͵��������
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void Robot_ParkingCheck(void)
{
	#define PUSH_PWM 500 //�Ƴ��ۼƵ�PWM��ֵ,������ִ�����
	
    //С����̬�����ж�
    float y_accle = (float)(imu.accel.y/1671.84f);//Y����ٶ�ʵ��ֵ
    float z_accle = (float)(imu.accel.z/1671.84f);//Z����ٶ�ʵ��ֵ
    float diff;

    //����Y��Z���ٶ��ں�ֵ����ֵԽ�ӽ�9.8����ʾС����̬Խˮƽ
    if( y_accle > 0 ) diff  = z_accle - y_accle;
    else diff  = z_accle + y_accle;
	
	if( robot.MOTOR_A.Target != 0 || robot.MOTOR_B.Target!=0 || \
		robot.MOTOR_C.Target != 0 || robot.MOTOR_D.Target!=0)
	{   //С����������,���õȴ���־λ.��λ����������
		park.wait_check = 1;  //�ȴ����
		park.timecore = 0;    //��λ�����ر���
		park.clear_state = 0;
		park.start_clear = 0;
	}
	else 
	{
		if( park.wait_check==1 ) //�ٶȷ�0 -> 0 �������
		{
			park.timecore++;
			if( park.timecore >= BALANCE_TASK_RATE*5 ) //С����ֹʱ�䳬��5��
			{
				park.wait_check=0;
				park.timecore = 0;
				
				//�������˲���б����,�����פ��ģʽ,��յ��������
				if( diff > 8.8f ) park.start_clear = 1 , park.clear_state = 0;
				else  park.clear_state |= 1<<7 ; //��б����,�������������.�����������
			}
		}
	}
	
	//����Ƿ���������.�������󽫻���С���Ƿ񱻶����ۿ�����(�Ƴ���Ϊ)
	if( ((park.clear_state>>7)&0x01)==1  )
	{
		if(diff > 8.8f )//С������б������Ϊǰ��.
		{
			//�����Ƴ���Ϊ
			if( abs(robot.MOTOR_A.Output) > PUSH_PWM || abs(robot.MOTOR_B.Output) > PUSH_PWM || \
				abs(robot.MOTOR_C.Output) > PUSH_PWM || abs(robot.MOTOR_D.Output) > PUSH_PWM)
			{
				park.timecore++;
				if( park.timecore >= BALANCE_TASK_RATE*10 ) //С��pwm�ۼ�ʱ�䳬��10��
				{
					park.timecore = 0;
					//����פ��ģʽ���������
					park.start_clear = 1 , park.clear_state = 0;
				}
			}
		}
	}
	
	//����פ��ģʽ,ִ���������
	if( park.start_clear==1 )
	{
		if( 1 == PI_Clear_Output(&PI_MotorA) ) park.clear_state |= 1<<0 ;
		else                                   park.clear_state &= ~(1<<0);
		
		if( 1 == PI_Clear_Output(&PI_MotorB) ) park.clear_state |= 1<<1 ;
		else                                   park.clear_state &= ~(1<<1);
		
		if( 1 == PI_Clear_Output(&PI_MotorC) ) park.clear_state |= 1<<2 ;
		else                                   park.clear_state &= ~(1<<2);
		
		if( 1 == PI_Clear_Output(&PI_MotorD) ) park.clear_state |= 1<<3 ;
		else                                   park.clear_state &= ~(1<<3);
		
		//4�����������
		if( (park.clear_state&0xff)==0x0f ) 
		{
			park.start_clear = 0;    //����������
			park.clear_state |= 1<<7;//���������
		}
	}
}

/**************************************************************************
Function function: Scan user buttons and perform different functions through different button states
Entry parameter: Frequency of executing scanning tasks
Return value: None
Author: SHEELTEC
�������ܣ�ɨ���û�����,ͨ����ͬ�İ���״ִ̬�в�ͬ�Ĺ���
��ڲ�����ִ��ɨ�������Ƶ��
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void UserKey_Scan(u16 rate)
{
	u8 state;
	
	//�����ϵ��û�����ɨ��
	state = KEY_Scan(rate,0);
	if(state==single_click) //����
	{
		charger.AllowRecharge = ! charger.AllowRecharge;
	}
	else if( state==double_click )//˫�����±궨imu��ֵ
	{	
		ImuData_copy(&imu.Deviation_gyro,&imu.Original_gyro);
		ImuData_copy(&imu.Deviation_accel,&imu.Original_accel);
	}
	
	else if( state==long_click ) //����
	{
		oled.page++;
		if( oled.page > oled.MAX_PAGE ) oled.page = 1;
	}
	
}

/**************************************************************************
Functionality: Check Automatic Recharge Station Status - Verifies the status of the automatic recharge station.
Input Parameters: None.
Return Value: None.
Author: WHEELTEC
�������ܣ����Զ��س��豸״̬���м��
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static void Charger_DevCheck(void)
{
	//�û����ý����Զ��س�״̬,��ʱ��ȷ��ϵͳ���Ƿ���ڸ��豸
	if( charger.AllowRecharge==1 )
		if( SysVal.HardWare_charger==0 ) charger.AllowRecharge=0,Find_Charging_HardWare();//�����ڳ��װ��Ӳ��,ͨ��������������ж���ȷ��
	
	//ϵͳ�ϴ��ڸ��豸ʱ,���Զ��س����Ӳ�����߼��
	if(SysVal.HardWare_charger==1)
	{   //�س�װ��Ӳ������,ִ�лس��豸������λ
		charger.OutLine_Check++;
		if( charger.OutLine_Check>RATE_100_HZ) auto_recharge_reset() , charger.OutLine_Check=BALANCE_TASK_RATE+1;//�����ĸ�����,�����ĸ�����Ƶ��,�����ǡ�BALANCE_TASK_RATE��
		
		//�����Զ��س��豸ʱ������⵽С��������,���յ����׮�ĺ����źţ��������п����Զ��س书��
		static u8 once=1;
		if( robot.voltage < 20.0f && charger.RED_STATE!=0 && once)
		{
			once = 0;
			charger.AllowRecharge = 1;
		}
	}
	
	//��ģң���źż��
	remoter.check_count++;//�˱����ں�ģ���Ž���ʱ�ᱻ��0
	if( remoter.check_count > Filter_Threshold )//�����趨����ֵ,˵�����޺�ģ�źŻ����Ǹ����ź�
	{
		remoter.check_count = Filter_Threshold + 1;
		remoter.ch1 = 1500;remoter.ch2=1500;remoter.ch3=1000;remoter.ch4=1500;//���Ż����ź�,����ͨ����ֵ��λ
	}
}

/*-------------------------------- Function auxiliary type related functions ------------------------------------*/
/*--------------------------------         ���ܸ�������غ���          ------------------------------------*/

/**************************************************************************
Function function: Save the steering variables of the Ackermann model's servo to Flash
Entry parameter: Flag bit for executing save
Return value: 1: Successfully saved 0: Not saved or failed to save
Author: WHEELTEC
�������ܣ����氢�������͵Ķ��ת�������Flash
��ڲ������Ƿ�ִ�б���ı�־λ
����  ֵ��1:����ɹ� 0:�����ڱ������� >1:flashд�����
��    �ߣ�WHEELTEC
**************************************************************************/
static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check=0;
	
	if(*flag==1)
	{
		*flag = 0;
		check = 1;
		taskENTER_CRITICAL();//����FLash�����ٽ磬��֤���ݰ�ȫ
		int buf[7]={0};
		buf[0] = Akm_Servo.Min;
		buf[1] = Akm_Servo.Mid;
		buf[2] = Akm_Servo.Max;
		buf[3] = robot_control.LineDiffParam;
		buf[4] = *((int32_t*)&robot_control.rc_speed);
		buf[5] = robot.V_KP;
		buf[6] = robot.V_KI;
		check += Write_Flash( (u32*)buf , 7);
		
		taskEXIT_CRITICAL();//�˳��ٽ�
		//��ȫ��д��ɹ�,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0);//��ȡ�±�Ϊ0������
	if( read!=0xffffffff ) Akm_Servo.Min = read;
	
	read = Read_Flash(1);//��ȡ�±�Ϊ1������
	if( read!=0xffffffff ) Akm_Servo.Mid = read;
	
	read = Read_Flash(2);//��ȡ�±�Ϊ2������
	if( read!=0xffffffff ) Akm_Servo.Max = read;
	
	read = Read_Flash(3); //��ƫϵ��
	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
	
	
	read = Read_Flash(4);//�ٶ�
	if( read!=0xffffffff ) robot_control.rc_speed = *((float*)&read);
	if( robot_control.rc_speed < 0 || robot_control.rc_speed > 10000 )//�쳣�ٶ����ݹ���
		robot_control.rc_speed = 500;
	
	
	read = Read_Flash(5);//KP����
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(read,-1,-1);
	}
	
	read = Read_Flash(6);//KI����
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(-1,read,-1);
	}
}

//static uint8_t Akm_SaveServo_Param(uint8_t *flag)
//{
//	u8 check=0;
//	
//	if(*flag==1)
//	{
//		*flag = 0;
//		check = 1;
//		taskENTER_CRITICAL();//����FLash�����ٽ磬��֤���ݰ�ȫ
//		int buf[4]={Akm_Servo.Min,Akm_Servo.Mid,Akm_Servo.Max,robot_control.LineDiffParam};
//		check += Write_Flash( (u32*)buf , 4);
//		taskEXIT_CRITICAL();//�˳��ٽ�
//		//��ȫ��д��ɹ�,check==1
//	}

//	return check;
//}

//void Akm_ReadServo_Param(void)
//{
//	int read;
//	read = Read_Flash(0);//��ȡ�±�Ϊ0������
//	if( read!=0xffffffff ) Akm_Servo.Min = read;
//	
//	read = Read_Flash(1);//��ȡ�±�Ϊ1������
//	if( read!=0xffffffff ) Akm_Servo.Mid = read;
//	
//	read = Read_Flash(2);//��ȡ�±�Ϊ2������
//	if( read!=0xffffffff ) Akm_Servo.Max = read;
//	
//	read = Read_Flash(3); //��ƫϵ��
//	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
//	
//}

/**************************************************************************
Functionality: Velocity Smoothing Function - Gradually adjusts the speed to the target speed using a specified step value.
Input Parameters: Current speed, target speed, smoothing step value.
Return Value: Smoothed speed.
Author: WHEELTEC
�������ܣ��ٶ�ƽ������,���ٶ������õĲ���ֵ����Ŀ���ٶ�
��ڲ�������ǰ�ٶȡ�Ŀ���ٶȡ�ƽ���Ĳ���ֵ
����  ֵ��ƽ������ٶ�
��    �ߣ�WHEELTEC
**************************************************************************/
static float Vel_SmoothControl(float now_speed,float targetSpeed,float step)
{
	if( now_speed > targetSpeed )
	{
		now_speed -= step;
		if( now_speed<=targetSpeed ) now_speed = targetSpeed;
	}
	else
	{
		now_speed += step;
		if( now_speed>=targetSpeed ) now_speed = targetSpeed;
	}

	return now_speed;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

static int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
�������ܣ��������������� �˲�
��ڲ������²ɼ�������
����  ֵ���˲��������
��    �ߣ�WHEELTEC
**************************************************************************/
#if defined AKM_CAR
#define FILTERING_TIMES 20
static int Slide_Mean_Filter(int data)
{
    u8 i;
    s32 Sum_Speed = 0;
    s16 Filter_Speed;
    static  s16 Speed_Buf[FILTERING_TIMES]= {0};
    for(i = 1 ; i<FILTERING_TIMES; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[FILTERING_TIMES - 1] =data;

    for(i = 0 ; i < FILTERING_TIMES; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
    return Filter_Speed;
}
#endif

///**************************************************************************
//Function: Smoothing the front wheel steering speed to prevent excessive steering gear current
//Input   : Current servo PWM, Target servo PWM, Smooth value
//Output  : none
//�������ܣ���ǰ��ת���ٶ���ƽ����������ֹ�����������
//��ڲ�������ǰ�������PWMֵ Ŀ��������PWMֵ ƽ��ֵ
//����  ֵ����
//**************************************************************************/
//int Smooth_steering(int currentPWM, int targetPWM, float step)
//{
//    int threshold=7;
//    if     (targetPWM>currentPWM+threshold) currentPWM+=step;
//    else if(targetPWM<currentPWM-threshold) currentPWM-=step;
//    else                                    currentPWM =targetPWM;

//    return currentPWM;
//}


/**************************************************************************
Functionality: Radian to Degree Conversion - Converts a given radian value to its corresponding degree value.
Input Parameters: Radian value.
Return Value: Degree value corresponding to the input radian.
Author: WHEELTEC
�������ܣ�����ת�Ƕ�
��ڲ���������
����  ֵ�����뻡�ȶ�Ӧ�ĽǶ�
��    �ߣ�WHEELTEC
**************************************************************************/
float rad_to_angle(const float rad)
{
    return rad/PI*180.0f;
}

/**************************************************************************
Functionality: Degree to Radian Conversion - Converts a given degree value to its corresponding radian value.
Input Parameters: Degree value.
Return Value: Radian value corresponding to the input degree.
Author: WHEELTEC
�������ܣ��Ƕ�ת����
��ڲ������Ƕ�
����  ֵ������Ƕȶ�Ӧ�Ļ���
��    �ߣ�WHEELTEC
**************************************************************************/
float angle_to_rad(const float angle)
{
    return angle/180.0f*PI;
}


/*-------------------------------- Software initialization related functions ------------------------------------*/
/*--------------------------------        ������ʼ����غ���          ------------------------------------*/
/**************************************************************************
Functionality: Incremental PI Controller Initialization - Initializes an incremental PI 
               (Proportional-Integral) controller with specified parameters.
Input Parameters: PI controller, kp value (proportional gain), ki value (integral gain).
Return Value: None.
Author: WHEELTEC
�������ܣ�����ʽPI��������ʼ��
��ڲ�����PI��������kpֵ��kiֵ
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void PI_Controller_Init(PI_CONTROLLER* p,int kp,int ki)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->LastestBias = 0;
	p->Output = 0;
	p->kp = kp;
	p->ki = ki;
	p->kd = 0;
}

//�����˿�����ر�����ʼ��
void ROBOT_CONTROL_t_Init(ROBOT_CONTROL_t* p)
{
	//�ɵ�����
	p->limt_max_speed = 3.5f;    //���������������˶��ٶ� m/s
	p->rc_speed = 500;           //������ң���ٶȻ�׼,��λ mm/s
	p->smooth_MotorStep = 0.02f; //�����˵���ٶ�ƽ������ֵ
	p->smooth_ServoStep = 20;    //�����˶���ٶ�ƽ������ֵ
	p->SoftWare_Stop = 0;        //����������ʧ��λ
	
	//���ٰ���������,������˶��ٶ���������Ϊ 6 m/s
	#if defined AKM_CAR
		if(robot.type==9) p->limt_max_speed = 6.0f;
		p->ServoLow_flag = 0;//���ٶ��ģʽĬ�Ϲر�
	#endif
	
	//ֻ������,����Ĭ��
	p->Vx = 0;
	p->Vy = 0;
	p->Vz = 0;
	p->smooth_Vx = 0;
	p->smooth_Vy = 0;
	p->smooth_Vz = 0;
	p->command_lostcount = 0;
	p->ControlMode = 1;
	p->FlagStop = 0;
	
	//��ƫϵ��
	p->LineDiffParam = 50;
}

//�Լ������ʼ��
static void ROBOT_SELFCHECK_t_Init(ROBOT_SELFCHECK_t* p)
{
	p->check_a = 0;
	p->check_b = 0;
	p->check_c = 0;
	p->check_d = 0;
	p->check_end = 0;
	p->errorflag  =0;
	p->DeepCheck = 0;
}

//���ĳ�������Ƿ����ı�
//��ڲ���:ִ�иú�����Ƶ�ʡ���Ҫ�������ݡ��仯���ȳ������ٱ�ʾ�仯
//����ֵ��1�����˸ı� 0:δ�ı�
uint8_t ValChangeCheck(const uint16_t rate,const short checkval,const uint8_t changeEva)
{
	static uint16_t timecore;
	static short lastval;
	
	const uint8_t DivisionFac = 2;     //��Ƶϵ��
	if( rate < DivisionFac ) return 0;//Ƶ�ʹ�С���޷����
	
	uint8_t changeflag = 0;//��ʾ�Ƿ��иı�

	timecore++;
	if( timecore >= rate/DivisionFac ) // 500ms���1��
	{
		timecore = 0;
		if( abs( lastval - checkval ) > changeEva )  changeflag = 1;
		else
		{
			changeflag=0;
		}
		lastval = checkval;
	}
	
	return changeflag ;
}

//����Լ����,��Ҫ��С�������������м��
static uint8_t Deep_SelfCheck( u16 RATE )
{	
	static uint32_t timecore = 0;
	static uint8_t errflag = 0;
	uint8_t check_ready = 0;
	
	//���������������������ӵı�����Ŀ��ֵ,�۲�����������.
	
	/*
	�������ȼ���
	1.���ָ���,˵���ǳ���ѡ��,���߱�����AB��ӷ������ǵ������˵������ӷ�
	2.������������ֵ,˵���ǵ����������ڽӴ�,�����Ǳ������ӿڽӴ�.
	3.������û��ֵ+PWM����,���Ǳ�����δ�ӻ����ǵ���ӿ�δ��
	*/

	timecore++;

	if( timecore==1 )
	{
		if( data_TaskHandle!=NULL ) vTaskSuspend(data_TaskHandle);//������Լ��������ʱ,�������ݷ�������(������ռ�ô���1),������Դ��ͻ
		any_printf(USART1,"\r\nС����������Լ�ģʽ,�����ת��˳��ΪA��B��C��D.��۲�ת������뱨����־.\r\n");
		Buzzer_AddTask(1,20);//��������ʾ��ʾ�ѽ�������Լ�ģʽ
	}
	
	//��0~1��ǰ����A���ٶ�
	if( timecore<RATE*2)
	{
		//ֱ��ָ�������Ŀ��ֵ��������Ӧ
		robot.MOTOR_A.Target = 0.5f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMB = 0;PWMC = 0;PWMD = 0;
	}
	else if( timecore==RATE*2 ) //���A���Ƿ�����쳣
	{
		any_printf(USART1,"================= ���A ================\r\n");
		
		//��ʼ�������
		if( robot.MOTOR_A.Encoder > 0.4f && robot.MOTOR_A.Encoder < 0.6f )
		{
			any_printf(USART1,"���A����.\r\n");
			//�������
			//A������
		}
		else
		{
			errflag = 1; //���ִ���
			
			//���������ȼ����������
			if( robot.MOTOR_A.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"��⵽A������������Դ���.\r\n");
				any_printf(USART1,"���飺\r\n1.����ѡ���Ƿ���ȷ\r\n2.���A���������Ƿ�����������\r\n3.���A�ı�����AB�������Ƿ����\r\n");
				//��Ӧ�ӿڳ��ָ�ֵ,���飺1.�����Ƿ���ȷ 2.������������Ƿ񷴽� 3.������AB��������Ƿ���ȷ
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Aʱ,��⵽�������ӿ�B���ź�.\r\n�Լ�ʱ���A�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��B�Ƿ���ڵ���\r\n��: 1.�����������A��B�ӿ��Ƿ���ڵ���\r\n");
				//������B��ֵ,���飺1.�������ӿ�A��ӿ�B�����Ƿ������ 2.��������A��B�ӿ��Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Aʱ,��⵽�������ӿ�C���ź�.\r\n�Լ�ʱ���A�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��C�Ƿ���ڵ���\r\n��: 1.�����������A��C�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������C��ֵ,���飺1.������A��C�ӿڽ����Ƿ������ 2.������A��C�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Aʱ,��⵽�������ӿ�D���ź�.\r\n�Լ�ʱ���A�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��D�Ƿ���ڵ���\r\n��: 1.�����������A��D�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������D��ֵ,���飺1.������A��D�ӿڽ����Ƿ������ 2.������A��D�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_A.Encoder) < 0.05f && PI_MotorA.Output > 5000 ) 
			{
				any_printf(USART1,"�������Aʱ,δ��⵽�������ź�.\r\n�Լ�ʱ���A�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A�Ƿ�����ɶ���δ��\r\n��: 1.�����A�ĵ�Դ���Ƿ�����ɶ���δ�� 2.���������½���ߵ�ţ�����Ƿ��ɶ���δ��\r\n");
				//������û��ֵ,��Ӧ�ӿ�PWMֵ�ϴ�,���飺1.�������ӿ��Ƿ��ɶ���δ�� 2.�����������ͷ�Ƿ��ɶ�����δ��
			}
			
		}
		
		any_printf(USART1,"================= ���A ================\r\n\r\n");
		
		//������,��λPI������,��ֹ��һ���������ܵ�����
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	else if( timecore>RATE*2 && timecore<RATE*4 ) //B�ֿ���
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.5f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMC = 0;PWMD = 0;
	} 
	else if( timecore==RATE*4 )  //B�ּ��
	{
		any_printf(USART1,"================= ���B ================\r\n");
		
		if( robot.MOTOR_B.Encoder > 0.4f && robot.MOTOR_B.Encoder < 0.6f )
		{
			any_printf(USART1,"���B����.\r\n");
		}
		else
		{
			errflag = 1; //���ִ���
			
			if( robot.MOTOR_B.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"��⵽B������������Դ���.\r\n");
				any_printf(USART1,"���飺\r\n1.����ѡ���Ƿ���ȷ\r\n2.���B���������Ƿ�����������\r\n3.���B�ı�����AB�������Ƿ����\r\n");
				//��Ӧ�ӿڳ��ָ�ֵ,���飺1.�����Ƿ���ȷ 2.������������Ƿ񷴽� 3.������AB��������Ƿ���ȷ
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Bʱ,��⵽�������ӿ�A���ź�.\r\n�Լ�ʱ���B�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��B�Ƿ���ڵ���\r\n��: 1.�����������A��B�ӿ��Ƿ���ڵ���\r\n");
				//������B��ֵ,���飺1.�������ӿ�B��ӿ�A�����Ƿ������ 2.��������B��A�ӿ��Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Bʱ,��⵽�������ӿ�C���ź�.\r\n�Լ�ʱ���B�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�B��C�Ƿ���ڵ���\r\n��: 1.�����������B��C�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������C��ֵ,���飺1.������B��C�ӿڽ����Ƿ������ 2.������B��C�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Bʱ,��⵽�������ӿ�D���ź�.\r\n�Լ�ʱ���B�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�B��D�Ƿ���ڵ���\r\n��: 1.�����������B��D�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������D��ֵ,���飺1.������B��D�ӿڽ����Ƿ������ 2.������B��D�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_B.Encoder) < 0.05f && PI_MotorB.Output > 5000 ) 
			{
				any_printf(USART1,"�������Bʱ,δ��⵽�������ź�.\r\n�Լ�ʱ���B�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�B�Ƿ�����ɶ���δ��\r\n��: 1.�����B�ĵ�Դ���Ƿ�����ɶ���δ�� 2.���������½���ߵ�ţ�����Ƿ��ɶ���δ��\r\n");
				//������û��ֵ,��Ӧ�ӿ�PWMֵ�ϴ�,���飺1.�������ӿ��Ƿ��ɶ���δ�� 2.�����������ͷ�Ƿ��ɶ�����δ��
			}
		}
		any_printf(USART1,"================= ���B ================\r\n\r\n");
		//������,��λPI������,��ֹ��һ���������ܵ�����
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#if defined OMNI_CAR || defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*4 && timecore<RATE*6 )//C�ֿ���
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.5f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMD = 0;
	} 
	else if( timecore==RATE*6 )//C�ּ��
	{
		any_printf(USART1,"================= ���C ================\r\n");
		
		if( robot.MOTOR_C.Encoder > 0.4f && robot.MOTOR_C.Encoder < 0.6f )
		{
			any_printf(USART1,"���C����.\r\n");
		}
		else
		{
			errflag = 1; //���ִ���
			
			if( robot.MOTOR_C.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"��⵽C������������Դ���.\r\n");
				any_printf(USART1,"���飺\r\n1.����ѡ���Ƿ���ȷ\r\n2.���C���������Ƿ�����������\r\n3.���C�ı�����AB�������Ƿ����\r\n");
				//��Ӧ�ӿڳ��ָ�ֵ,���飺1.�����Ƿ���ȷ 2.������������Ƿ񷴽� 3.������AB��������Ƿ���ȷ
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Cʱ,��⵽�������ӿ�A���ź�.\r\n�Լ�ʱ���C�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��C�Ƿ���ڵ���\r\n��: 1.�����������A��C�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������B��ֵ,���飺1.�������ӿ�C��ӿ�A�����Ƿ������ 2.��������C��A�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Cʱ,��⵽�������ӿ�B���ź�.\r\n�Լ�ʱ���C�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�B��C�Ƿ���ڵ���\r\n��: 1.�����������B��C�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������C��ֵ,���飺1.������C��B�ӿڽ����Ƿ������ 2.������C��B�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Cʱ,��⵽�������ӿ�D���ź�.\r\n�Լ�ʱ���C�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�C��D�Ƿ���ڵ���\r\n��: 1.�����������C��D�ӿ��Ƿ���ڵ���\r\n");
				//������D��ֵ,���飺1.������C��D�ӿڽ����Ƿ������ 2.������C��D�ӿ��Ƿ���ڵ��� 
			}
			else if( fabs(robot.MOTOR_C.Encoder) < 0.05f && PI_MotorC.Output > 5000 ) 
			{
				any_printf(USART1,"�������Cʱ,δ��⵽�������ź�.\r\n�Լ�ʱ���C�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�C�Ƿ�����ɶ���δ��\r\n��: 1.�����C�ĵ�Դ���Ƿ�����ɶ���δ�� 2.���������½��ұߵ�ţ�����Ƿ��ɶ���δ��\r\n");
				//������û��ֵ,��Ӧ�ӿ�PWMֵ�ϴ�,���飺1.�������ӿ��Ƿ��ɶ���δ�� 2.�����������ͷ�Ƿ��ɶ�����δ��
			}
		}
		any_printf(USART1,"================= ���C ================\r\n\r\n");
		//������,��λPI������,��ֹ��һ���������ܵ�����
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#endif /* if defined OMNI_CAR _4WD_CAR MEC_CAR */
	
	#if defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*6 && timecore<RATE*8 )//D�ֿ���
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.5f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMC = 0;
	}
	else if( timecore==RATE*8  )//D�ּ��
	{
		any_printf(USART1,"================= ���D ================\r\n");
		
		if( robot.MOTOR_D.Encoder > 0.4f && robot.MOTOR_D.Encoder < 0.6f )
		{
			any_printf(USART1,"���D����.\r\n");
		}
		else
		{
			errflag = 1; //���ִ���
			
			if( robot.MOTOR_D.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"��⵽D������������Դ���.\r\n");
				any_printf(USART1,"���飺\r\n1.����ѡ���Ƿ���ȷ\r\n2.���D���������Ƿ�����������\r\n3.���D�ı�����AB�������Ƿ����\r\n");
				//��Ӧ�ӿڳ��ָ�ֵ,���飺1.�����Ƿ���ȷ 2.������������Ƿ񷴽� 3.������AB��������Ƿ���ȷ
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Dʱ,��⵽�������ӿ�A���ź�.\r\n�Լ�ʱ���D�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�A��D�Ƿ���ڵ���\r\n��: 1.�����������A��D�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������B��ֵ,���飺1.�������ӿ�D��ӿ�A�����Ƿ������ 2.��������D��A�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Dʱ,��⵽�������ӿ�B���ź�.\r\n�Լ�ʱ���D�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�B��D�Ƿ���ڵ���\r\n��: 1.�����������B��D�ӿ��Ƿ���ڵ��� 2.���������½ǵ�����ţ�����Ƿ���ڵ���\r\n");
				//������C��ֵ,���飺1.������D��B�ӿڽ����Ƿ������ 2.������D��B�ӿ��Ƿ���ڵ��� 3.���ӵ�ţ�����Ƿ���ڵ���
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"�������Dʱ,��⵽�������ӿ�C���ź�.\r\n�Լ�ʱ���D�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�C��D�Ƿ���ڵ���\r\n��: 1.�����������C��D�ӿ��Ƿ���ڵ���\r\n");
				//������D��ֵ,���飺1.������C��D�ӿڽ����Ƿ������ 2.������C��D�ӿ��Ƿ���ڵ��� 
			}
			else if( fabs(robot.MOTOR_D.Encoder) < 0.05f && PI_MotorD.Output > 5000 ) 
			{
				any_printf(USART1,"�������Dʱ,δ��⵽�������ź�.\r\n�Լ�ʱ���D�Ƿ�ת��?\r\n");
				any_printf(USART1,"��: 1.���������ӿ�D�Ƿ�����ɶ���δ��\r\n��: 1.�����D�ĵ�Դ���Ƿ�����ɶ���δ�� 2.���������½��ұߵ�ţ�����Ƿ��ɶ���δ��\r\n");
				//������û��ֵ,��Ӧ�ӿ�PWMֵ�ϴ�,���飺1.�������ӿ��Ƿ��ɶ���δ�� 2.�����������ͷ�Ƿ��ɶ�����δ��
			}
		}
		
		any_printf(USART1,"================= ���D ================\r\n");
		
		//������,��λPI������,��ֹ��һ���������ܵ�����
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	#endif /* if defined  _4WD_CAR MEC_CAR */
	
	else
	{
		check_ready = 1;
		robot.MOTOR_A.Target = 0.0f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		Set_Pwm(0,0,0,0,0);
		any_printf(USART1,"\r\n\r\n");
		Buzzer_AddTask(1,20);//��������ʾ��ʾ���������Լ�
		
		//����Լ�������ޱ���,�ָ����������.
		if( 0 == errflag  )
			if( data_TaskHandle!=NULL ) vTaskResume(data_TaskHandle);
		
		timecore = 0;
		errflag = 0;
	}

	return check_ready;
	
}
