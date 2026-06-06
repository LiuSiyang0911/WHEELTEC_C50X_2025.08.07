#include "wheeltec_robot.h"
#include "Quaternion_Solution.h"
#include <vector>

sensor_msgs::Imu Mpu6050;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheeltec_robot");
  turn_on_robot Robot_Control;
  Robot_Control.Control();
  return 0;
}

short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return (transition_16 / 1000) + (transition_16 % 1000) * 0.001f;
}

uint8_t turn_on_robot::Check_Buffer_BCC(const uint8_t *data, size_t length) const
{
  uint8_t bcc = 0;
  for(size_t i = 0; i < length; ++i)
  {
    bcc ^= data[i];
  }
  return bcc;
}

int16_t turn_on_robot::Read_Int16_BE(const uint8_t *data) const
{
  return static_cast<int16_t>(Read_Uint16_BE(data));
}

uint16_t turn_on_robot::Read_Uint16_BE(const uint8_t *data) const
{
  return (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
}

int32_t turn_on_robot::Read_Int32_BE(const uint8_t *data) const
{
  return static_cast<int32_t>(Read_Uint32_BE(data));
}

uint32_t turn_on_robot::Read_Uint32_BE(const uint8_t *data) const
{
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  short transition;

  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  transition = twist_aux.linear.x * 1000;
  Send_Data.tx[4] = transition;
  Send_Data.tx[3] = transition >> 8;

  transition = twist_aux.linear.y * 1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition >> 8;

  transition = twist_aux.angular.z * 1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition >> 8;

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
  Send_Data.tx[10] = FRAME_TAIL;
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  }
  catch(serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port");
  }
}

void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = ros::Time::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id;
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
  imu_publisher.publish(Imu_Data_Pub);
}

void turn_on_robot::Publish_Odom()
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = odom_frame_id;
  odom.pose.pose.position.x = Robot_Pos.X;
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = Robot_Pos.Z;
  odom.pose.pose.orientation = odom_quat;
  odom.child_frame_id = robot_frame_id;
  odom.twist.twist.linear.x = Robot_Vel.X;
  odom.twist.twist.linear.y = Robot_Vel.Y;
  odom.twist.twist.angular.z = Robot_Vel.Z;

  if(Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  }
  else
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  }
  odom_publisher.publish(odom);
}

void turn_on_robot::Publish_Voltage()
{
  std_msgs::Float32 voltage_msgs;
  static float Count_Voltage_Pub = 0;
  if(Count_Voltage_Pub++ > 10)
  {
    Count_Voltage_Pub = 0;
    voltage_msgs.data = Power_voltage;
    voltage_publisher.publish(voltage_msgs);
  }
}

void turn_on_robot::Publish_AkmState(const ros::Time &stamp)
{
  turn_on_wheeltec_robot::AkmState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = robot_frame_id;
  msg.seq_id = Akm_Ext_Data.seq_id;
  msg.control_tick_us = Akm_Ext_Data.control_tick_us;
  msg.dt_us = Akm_Ext_Data.dt_us;
  msg.left_encoder_delta = Akm_Ext_Data.left_encoder_delta;
  msg.right_encoder_delta = Akm_Ext_Data.right_encoder_delta;
  msg.left_wheel_speed = Akm_Ext_Data.left_wheel_speed;
  msg.right_wheel_speed = Akm_Ext_Data.right_wheel_speed;
  msg.steering_feedback_raw = Akm_Ext_Data.steering_feedback_raw;
  msg.steering_target_raw = Akm_Ext_Data.steering_target_raw;
  msg.steering_angle = Akm_Ext_Data.steering_angle;
  msg.steering_pwm = Akm_Ext_Data.steering_pwm;
  msg.status_flags = Akm_Ext_Data.status_flags;
  msg.control_mode = Akm_Ext_Data.control_mode;
  msg.robot_type = Akm_Ext_Data.robot_type;
  akm_state_publisher.publish(msg);
}

void turn_on_robot::Publish_ControlDebug(const ros::Time &stamp)
{
  turn_on_wheeltec_robot::ControlDebug msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = robot_frame_id;
  msg.seq_id = Akm_Ext_Data.seq_id;
  msg.control_tick_us = Akm_Ext_Data.control_tick_us;
  msg.target_vx = Akm_Ext_Data.target_vx;
  msg.target_vy = Akm_Ext_Data.target_vy;
  msg.target_vz = Akm_Ext_Data.target_vz;
  msg.legacy_vx = Robot_Vel.X;
  msg.legacy_vy = Robot_Vel.Y;
  msg.legacy_vz = Robot_Vel.Z;
  msg.motor_left_pwm = Akm_Ext_Data.motor_left_pwm;
  msg.motor_right_pwm = Akm_Ext_Data.motor_right_pwm;
  msg.steering_pwm = Akm_Ext_Data.steering_pwm;
  msg.status_flags = Akm_Ext_Data.status_flags;
  msg.control_mode = Akm_Ext_Data.control_mode;
  control_debug_publisher.publish(msg);
}

void turn_on_robot::Publish_ChassisDiagnostics(const ros::Time &stamp)
{
  turn_on_wheeltec_robot::ChassisDiagnostics msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = robot_frame_id;
  msg.seq_id = Akm_Ext_Data.seq_id;
  msg.control_tick_us = Akm_Ext_Data.control_tick_us;
  msg.battery_voltage = Power_voltage;
  msg.flag_stop = (Akm_Ext_Data.status_flags & 0x0001) != 0;
  msg.command_timeout = (Akm_Ext_Data.status_flags & 0x0002) != 0;
  msg.low_voltage = (Akm_Ext_Data.status_flags & 0x0004) != 0;
  msg.self_check_error = (Akm_Ext_Data.status_flags & 0x0010) != 0;
  msg.steering_angle_valid = (Akm_Ext_Data.status_flags & 0x0020) != 0;
  msg.status_flags = Akm_Ext_Data.status_flags;
  msg.packet_drop_count = akm_packet_drop_count;
  msg.checksum_error_count = akm_checksum_error_count;
  msg.legacy_error_count = akm_legacy_error_count;
  diagnostics_publisher.publish(msg);
}

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0;

  if(mode == READ_DATA_CHECK)
  {
    for(unsigned char k = 0; k < Count_Number; k++)
    {
      check_sum ^= Receive_Data.rx[k];
    }
  }
  if(mode == SEND_DATA_CHECK)
  {
    for(unsigned char k = 0; k < Count_Number; k++)
    {
      check_sum ^= Send_Data.tx[k];
    }
  }
  return check_sum;
}

bool turn_on_robot::Decode_Legacy_Frame(const uint8_t *frame)
{
  short transition_16 = 0;

  if(frame[0] != FRAME_HEADER || frame[RECEIVE_DATA_SIZE - 1] != FRAME_TAIL)
  {
    return false;
  }
  if(Check_Buffer_BCC(frame, 22) != frame[22])
  {
    return false;
  }

  memcpy(Receive_Data.rx, frame, RECEIVE_DATA_SIZE);
  Receive_Data.Frame_Header = Receive_Data.rx[0];
  Receive_Data.Frame_Tail = Receive_Data.rx[23];
  Receive_Data.Flag_Stop = Receive_Data.rx[1];
  Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
  Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
  Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

  Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
  Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
  Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
  Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
  Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
  Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

  Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
  Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
  Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
  Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
  Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
  Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

  transition_16 |= Receive_Data.rx[20] << 8;
  transition_16 |= Receive_Data.rx[21];
  Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001f;
  return true;
}

bool turn_on_robot::Decode_Akm_Extension(const uint8_t *frame)
{
  const uint8_t *payload = &frame[4];

  if(frame[0] != AKM_EXT_FRAME_HEADER ||
     frame[1] != AKM_EXT_FRAME_TYPE_MOTION ||
     frame[2] != AKM_EXT_PROTOCOL_VERSION ||
     frame[3] != AKM_EXT_PAYLOAD_SIZE ||
     frame[AKM_EXT_FRAME_SIZE - 1] != AKM_EXT_FRAME_TAIL)
  {
    return false;
  }
  if(Check_Buffer_BCC(frame, AKM_EXT_FRAME_SIZE - 2) != frame[AKM_EXT_FRAME_SIZE - 2])
  {
    akm_checksum_error_count++;
    return false;
  }

  Akm_Ext_Data.seq_id = Read_Uint16_BE(&payload[0]);
  Akm_Ext_Data.control_tick_us = Read_Uint32_BE(&payload[2]);
  Akm_Ext_Data.dt_us = Read_Uint16_BE(&payload[6]);
  memcpy(Akm_Ext_Data.legacy_rx, &payload[8], RECEIVE_DATA_SIZE);

  if(!Decode_Legacy_Frame(Akm_Ext_Data.legacy_rx))
  {
    akm_legacy_error_count++;
    return false;
  }

  if(has_last_ext_seq)
  {
    uint16_t expected = static_cast<uint16_t>(last_ext_seq_id + 1);
    if(Akm_Ext_Data.seq_id != expected)
    {
      uint16_t missing = static_cast<uint16_t>(Akm_Ext_Data.seq_id - expected);
      akm_packet_drop_count += (missing > 0 && missing < 32768) ? missing : 1;
    }
  }
  last_ext_seq_id = Akm_Ext_Data.seq_id;
  has_last_ext_seq = true;

  Akm_Ext_Data.left_encoder_delta = Read_Int32_BE(&payload[32]);
  Akm_Ext_Data.right_encoder_delta = Read_Int32_BE(&payload[36]);
  Akm_Ext_Data.left_wheel_speed = Read_Int16_BE(&payload[40]) * 0.001f;
  Akm_Ext_Data.right_wheel_speed = Read_Int16_BE(&payload[42]) * 0.001f;
  Akm_Ext_Data.steering_feedback_raw = Read_Int16_BE(&payload[44]);
  Akm_Ext_Data.steering_target_raw = Read_Int16_BE(&payload[46]);
  Akm_Ext_Data.steering_angle = Read_Int16_BE(&payload[48]) * 0.0001f;
  Akm_Ext_Data.steering_pwm = Read_Int16_BE(&payload[50]);
  Akm_Ext_Data.motor_left_pwm = Read_Int16_BE(&payload[52]);
  Akm_Ext_Data.motor_right_pwm = Read_Int16_BE(&payload[54]);
  Akm_Ext_Data.target_vx = Read_Int16_BE(&payload[56]) * 0.001f;
  Akm_Ext_Data.target_vy = Read_Int16_BE(&payload[58]) * 0.001f;
  Akm_Ext_Data.target_vz = Read_Int16_BE(&payload[60]) * 0.001f;
  Akm_Ext_Data.status_flags = Read_Uint16_BE(&payload[62]);
  Akm_Ext_Data.control_mode = payload[64];
  Akm_Ext_Data.robot_type = payload[65];
  Akm_Ext_Data.valid = true;
  return true;
}

bool turn_on_robot::Get_Sensor_Data_Extended()
{
  try
  {
    size_t available = Stm32_Serial.available();
    if(available == 0)
    {
      uint8_t one_byte = 0;
      if(Stm32_Serial.read(&one_byte, 1) == 1)
      {
        serial_rx_cache.push_back(one_byte);
      }
    }
    else
    {
      vector<uint8_t> bytes;
      bytes.resize(available);
      size_t read_len = Stm32_Serial.read(&bytes[0], available);
      for(size_t i = 0; i < read_len; ++i)
      {
        serial_rx_cache.push_back(bytes[i]);
      }
    }
  }
  catch(serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to read data through serial port");
    return false;
  }

  while(serial_rx_cache.size() > 512)
  {
    serial_rx_cache.pop_front();
  }

  while(!serial_rx_cache.empty())
  {
    uint8_t header = serial_rx_cache.front();
    if(header != AKM_EXT_FRAME_HEADER && header != FRAME_HEADER)
    {
      serial_rx_cache.pop_front();
      continue;
    }

    if(header == AKM_EXT_FRAME_HEADER)
    {
      if(serial_rx_cache.size() < AKM_EXT_FRAME_SIZE) return false;
      uint8_t frame[AKM_EXT_FRAME_SIZE];
      for(size_t i = 0; i < AKM_EXT_FRAME_SIZE; ++i) frame[i] = serial_rx_cache[i];
      if(Decode_Akm_Extension(frame))
      {
        for(size_t i = 0; i < AKM_EXT_FRAME_SIZE; ++i) serial_rx_cache.pop_front();
        return true;
      }
      serial_rx_cache.pop_front();
      continue;
    }

    if(serial_rx_cache.size() < RECEIVE_DATA_SIZE) return false;
    uint8_t frame[RECEIVE_DATA_SIZE];
    for(size_t i = 0; i < RECEIVE_DATA_SIZE; ++i) frame[i] = serial_rx_cache[i];
    Akm_Ext_Data.valid = false;
    if(Decode_Legacy_Frame(frame))
    {
      for(size_t i = 0; i < RECEIVE_DATA_SIZE; ++i) serial_rx_cache.pop_front();
      return true;
    }
    serial_rx_cache.pop_front();
  }

  return false;
}

bool turn_on_robot::Get_Sensor_Data_New()
{
  return Get_Sensor_Data_Extended();
}

bool turn_on_robot::Get_Sensor_Data()
{
  return Get_Sensor_Data_Extended();
}

void turn_on_robot::Control()
{
  _Last_Time = ros::Time::now();
  while(ros::ok())
  {
    _Now = ros::Time::now();
    Sampling_Time = (_Now - _Last_Time).toSec();

    if(Get_Sensor_Data_Extended())
    {
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

      Publish_Odom();
      Publish_ImuSensor();
      Publish_Voltage();

      if(Akm_Ext_Data.valid)
      {
        Publish_AkmState(_Now);
        Publish_ControlDebug(_Now);
        Publish_ChassisDiagnostics(_Now);
      }

      _Last_Time = _Now;
    }

    ros::spinOnce();
  }
}

turn_on_robot::turn_on_robot()
  : Sampling_Time(0),
    Power_voltage(0),
    akm_packet_drop_count(0),
    akm_checksum_error_count(0),
    akm_legacy_error_count(0),
    last_ext_seq_id(0),
    has_last_ext_seq(false)
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
  memset(&Akm_Ext_Data, 0, sizeof(Akm_Ext_Data));

  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/wheeltec_controller");
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);
  private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom_combined");
  private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_footprint");
  private_nh.param<std::string>("gyro_frame_id", gyro_frame_id, "gyro_link");

  voltage_publisher = n.advertise<std_msgs::Float32>("PowerVoltage", 10);
  odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);
  imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 20);
  akm_state_publisher = n.advertise<turn_on_wheeltec_robot::AkmState>("wheeltec/akm_state", 50);
  control_debug_publisher = n.advertise<turn_on_wheeltec_robot::ControlDebug>("wheeltec/control_debug", 50);
  diagnostics_publisher = n.advertise<turn_on_wheeltec_robot::ChassisDiagnostics>("wheeltec/chassis_diagnostics", 10);

  Cmd_Vel_Sub = n.subscribe("cmd_vel", 1, &turn_on_robot::Cmd_Vel_Callback, this);

  ROS_INFO_STREAM("Data ready");

  try
  {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
    Stm32_Serial.flushInput();
  }
  catch(serial::IOException& e)
  {
    ROS_ERROR_STREAM("wheeltec_robot can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("wheeltec_robot serial port opened");
  }
}

turn_on_robot::~turn_on_robot()
{
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
  Send_Data.tx[10] = FRAME_TAIL;
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  }
  catch(serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port");
  }
  Stm32_Serial.close();
  ROS_INFO_STREAM("Shutting down");
}
