//
// Created by zhouz on 2026/3/15.
//

#ifndef BMI088_DEMO_FILTERING_H
#define BMI088_DEMO_FILTERING_H

#define PI 3.14159265358979323846f
// 补充：弧度/角度转换常量（1° = π/180 rad）
#define DEG_TO_RAD 0.017453292519943295f

// 首先是一些宏定义，用于数据转换
#define G 9.80665f          // 重力加速度标准值

#define Kp 1.50f            // 互补滤波比例系数
#define Ki 0.005f           // 互补滤波积分系数
#define halfT 0.0025f       // 半采样周期，单位秒。假设采样频率为200Hz，周期0.005s，halfT=0.0025

extern float Yaw, Pitch, Roll;

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

float invSqrt(float x);


#endif //BMI088_DEMO_FILTERING_H