#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//�����˽ṹ��ز���
typedef struct  
{
	float WheelSpacing;           //Wheelspacing, Mec_Car is half wheelspacing //�־���� ���ֳ�Ϊ���־�
	float AxleSpacing;            //Axlespacing, Mec_Car is half axlespacing   //������ ���ֳ�Ϊ�����	
	float Wheel_Circ;             //Wheel circumference                        //�������ܳ�
	float GearRatio;              //Motor_gear_ratio                            //������ٱȲ���
	uint16_t   EncoderAccuracy;   //Number_of_encoder_lines                     //����������(����������)
	uint8_t    type;              //Robot model                                 //�ͺ�
	
	//��ͬ���ͳ���ר������
	#if defined AKM_CAR
		float MIN_turn_radius;    //������С������Сת��뾶����
	#endif
	
	#if defined OMNI_CAR
		float TurnRadiaus;        //Rotation radius of omnidirectional trolley //ȫ����С����ת�뾶
	#endif
	
}Robot_Parament_InitTypeDef;

//Robot servo motor related parameters
//�������ŷ������ز���
typedef struct  
{
	float Target;                //Control the target speed of the motor            //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Encoder;               //Read the real time speed of the motor by encoder //��������ֵ����ȡ���ʵʱ�ٶ�
}Moto_parameter;


typedef struct
{
	Robot_Parament_InitTypeDef HardwareParam;//�����˵Ľṹ����
	
	Moto_parameter MOTOR_A;
	Moto_parameter MOTOR_B;

	#if defined AKM_CAR
		Moto_parameter MOTOR_SERVO;
	#elif defined OMNI_CAR
		Moto_parameter MOTOR_C;
	#elif defined MEC_CAR || defined _4WD_CAR
		Moto_parameter MOTOR_C;
		Moto_parameter MOTOR_D;
	#endif
	
	//���ڿ��ƻ������˶���pid����
	int V_KP;
	int V_KI;
	
}ROBOT;


//Motor_gear_ratio
//������ٱ�
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47

//Number_of_encoder_lines
//����������
#define		GMR_500  500
#define	    Hall_13  13

//The encoder octave depends on the encoder initialization Settings
//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples 4

//Ĭ��PID����
#define VEL_KP 0
#define VEL_KI 0

//��ͬ������µĲ���
#if defined AKM_CAR
	//������С��ӵ�еĳ�������
	#define CAR_NUMBER    10    
	//Wheel_spacing //�־�
	#define   SENIOR_AKM_wheelspacing  0.322f //���䰢����
	#define   TOP_AKM_BS_wheelspacing  0.508f //�����ʽ���Ұ�����
	#define   TOP_AKM_DL_wheelspacing  0.585f //����������Ұ�����
	
	//Axle_spacing //���
	#define   SENIOR_AKM_axlespacing   0.322f //���䰢����
	#define   TOP_AKM_BS_axlespacing   0.590f //�����ʽ���Ұ�����
	#define   TOP_AKM_DL_axlespacing   0.530f //����������Ұ�����
	
	//Diameter of trolley tire
	//С����ֱ̥��
	#define   SENIOR_AKM_WheelDiameter  0.125 //���䰢����
	#define   TOP_AKM_BS_WheelDiameter  0.180 //�����ʽ���Ұ�����
	#define   TOP_AKM_DL_WheelDiameter  0.254 //����������Ұ�����

	//The minimum turning radius of different Ackermann models is determined by the mechanical structure:
	//the maximum Angle of the wheelbase, wheelbase and front wheel
	//��ͬ���������͵���Сת��뾶���ɻ�е�ṹ�������־ࡢ��ࡢǰ�����ת��
	#define   SENIOR_AKM_MIN_TURN_RADIUS  0.750f //���䰢����
	#define   TOP_AKM_BS_MIN_TURN_RADIUS  1.400f //�����ʽ���Ұ�����
	#define   TOP_AKM_DL_MIN_TURN_RADIUS  1.200f //����������Ұ�����
	
#elif defined DIFF_CAR
	//����С���ĳ�������
	#define   CAR_NUMBER    9   
	
	//�־�
	#define   TOP_DIFF_wheelspacing            0.329f //���ٳ�
	#define   FOUR_WHEEL_DIFF_BS_wheelspacing  0.573f //���ֲ��ٰ�ʽ����
	#define   FOUR_WHEEL_DIFF_DL_wheelspacing  0.573f //���ֲ��ٶ�������
	
	//���ٳ����������
	
	//��ֱ̥��
	#define   TOP_DIFF_WheelDiameter        0.125 //���ֲ���������ֱ��
	#define   FOUR_WHEEL_DIFF_WheelDiameter 0.215 //���ֲ���������ֱ��
	
#elif defined MEC_CAR

	#define CAR_NUMBER    11     
	
#elif defined _4WD_CAR

	#define CAR_NUMBER    12    
	
#elif defined OMNI_CAR

	#define CAR_NUMBER    7     
	
#endif




#endif
