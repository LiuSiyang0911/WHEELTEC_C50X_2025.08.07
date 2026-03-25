#ifndef __FILTER_H
#define __FILTER_H
#include "system.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
// 1D卡尔曼滤波结构体定义
typedef struct {
    float lastP; // 上次估算协方差
    float nowP;  // 当前估算协方差
    float out;   // 卡尔曼输出
    float kg;    // 卡尔曼增益
    float Q;     // 过程噪声协方差
    float R;     // 测量噪声协方差
} Kalman_TypeDef;

/**
 * @brief 一阶卡尔曼滤波函数
 * @param k 卡尔曼结构体指针
 * @param input 待滤波的原始数据
 * @return 滤波后的数据
 */
float Kalman_Filter_1D(Kalman_TypeDef *k, float input);

// Alpha-Beta滤波结构体定义
typedef struct {
    float x;     // 当前估计值
    float v;     // 当前变化率估计值
    float alpha; // 测量修正系数
    float beta;  // 变化率修正系数
    float dt;    // 采样周期(s)
    uint8_t initialized;
} AlphaBeta_Filter_t;

void AlphaBeta_Filter_Reset(AlphaBeta_Filter_t *f, float value);
float AlphaBeta_Filter_Update(AlphaBeta_Filter_t *f, float input);

// 一阶低通滤波结构体定义
typedef struct {
    float alpha; // 离散低通系数
    float y;     // 当前输出
    uint8_t initialized;
} FirstOrder_LPF_t;

void FirstOrder_LPF_Reset(FirstOrder_LPF_t *f, float value);
float FirstOrder_LPF_Update(FirstOrder_LPF_t *f, float input);
#endif
