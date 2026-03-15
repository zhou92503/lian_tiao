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

// 全局四元数，初始化为无旋转状态（代表机体坐标系与地理坐标系对齐）
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// 误差积分项，用于消除稳态误差
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
// 弧度转角度

#endif //BMI088_DEMO_FILTERING_H