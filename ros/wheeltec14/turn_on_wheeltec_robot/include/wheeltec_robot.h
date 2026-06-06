#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "ros/ros.h"
#include <deque>
#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include "turn_on_wheeltec_robot/AkmState.h"
#include "turn_on_wheeltec_robot/ControlDebug.h"
#include "turn_on_wheeltec_robot/ChassisDiagnostics.h"
using namespace std;

#define SEND_DATA_CHECK   1
#define READ_DATA_CHECK   0
#define FRAME_HEADER      0X7B
#define FRAME_TAIL        0X7D
#define RECEIVE_DATA_SIZE 24
#define SEND_DATA_SIZE    11
#define PI                3.1415926f

#define AKM_EXT_FRAME_HEADER      0X7E
#define AKM_EXT_FRAME_TAIL        0X7F
#define AKM_EXT_FRAME_TYPE_MOTION 0X01
#define AKM_EXT_PROTOCOL_VERSION  0X01
#define AKM_EXT_PAYLOAD_SIZE      66
#define AKM_EXT_FRAME_SIZE        72

#define GYROSCOPE_RATIO   0.00026644f
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::Imu Mpu6050;

const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9};

typedef struct __Vel_Pos_Data_
{
  float X;
  float Y;
  float Z;
} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
  short accele_x_data;
  short accele_y_data;
  short accele_z_data;
  short gyros_x_data;
  short gyros_y_data;
  short gyros_z_data;
} MPU6050_DATA;

typedef struct _SEND_DATA_
{
  uint8_t tx[SEND_DATA_SIZE];
  float X_speed;
  float Y_speed;
  float Z_speed;
  unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t Flag_Stop;
  unsigned char Frame_Header;
  float X_speed;
  float Y_speed;
  float Z_speed;
  float Power_Voltage;
  unsigned char Frame_Tail;
} RECEIVE_DATA;

typedef struct _AKM_EXTENSION_DATA_
{
  uint16_t seq_id;
  uint32_t control_tick_us;
  uint16_t dt_us;
  uint8_t legacy_rx[RECEIVE_DATA_SIZE];
  int32_t left_encoder_delta;
  int32_t right_encoder_delta;
  float left_wheel_speed;
  float right_wheel_speed;
  int16_t steering_feedback_raw;
  int16_t steering_target_raw;
  float steering_angle;
  int16_t steering_pwm;
  int16_t motor_left_pwm;
  int16_t motor_right_pwm;
  float target_vx;
  float target_vy;
  float target_vz;
  uint16_t status_flags;
  uint8_t control_mode;
  uint8_t robot_type;
  bool valid;
} AKM_EXTENSION_DATA;

class turn_on_robot
{
public:
  turn_on_robot();
  ~turn_on_robot();
  void Control();
  serial::Serial Stm32_Serial;

private:
  ros::NodeHandle n;
  ros::Time _Now, _Last_Time;
  float Sampling_Time;

  ros::Subscriber Cmd_Vel_Sub;
  void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);

  ros::Publisher odom_publisher, imu_publisher, voltage_publisher;
  ros::Publisher akm_state_publisher, control_debug_publisher, diagnostics_publisher;
  void Publish_Odom();
  void Publish_ImuSensor();
  void Publish_Voltage();
  void Publish_AkmState(const ros::Time &stamp);
  void Publish_ControlDebug(const ros::Time &stamp);
  void Publish_ChassisDiagnostics(const ros::Time &stamp);

  bool Get_Sensor_Data();
  bool Get_Sensor_Data_New();
  bool Get_Sensor_Data_Extended();
  bool Decode_Legacy_Frame(const uint8_t *frame);
  bool Decode_Akm_Extension(const uint8_t *frame);
  unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);
  uint8_t Check_Buffer_BCC(const uint8_t *data, size_t length) const;
  short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);
  float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);
  int16_t Read_Int16_BE(const uint8_t *data) const;
  uint16_t Read_Uint16_BE(const uint8_t *data) const;
  int32_t Read_Int32_BE(const uint8_t *data) const;
  uint32_t Read_Uint32_BE(const uint8_t *data) const;

  string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
  int serial_baud_rate;
  RECEIVE_DATA Receive_Data;
  SEND_DATA Send_Data;
  AKM_EXTENSION_DATA Akm_Ext_Data;

  Vel_Pos_Data Robot_Pos;
  Vel_Pos_Data Robot_Vel;
  MPU6050_DATA Mpu6050_Data;
  float Power_voltage;

  deque<uint8_t> serial_rx_cache;
  uint32_t akm_packet_drop_count;
  uint32_t akm_checksum_error_count;
  uint32_t akm_legacy_error_count;
  uint16_t last_ext_seq_id;
  bool has_last_ext_seq;
};
#endif
