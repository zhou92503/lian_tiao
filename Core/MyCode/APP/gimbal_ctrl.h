//
// Created by zhouz on 2026/3/15.
//

#ifndef BMI088_DEMO_GIMBAL_H
#define BMI088_DEMO_GIMBAL_H

#define YAW_MOTOR_ID 0
#define PITCH_MOTOR_ID  1

#include "motor.h"


void Gimbal_PID_Control(void);
void Gimbal_Init(void);



typedef struct
{
    uint16_t angle;  //机械角度 0~8191
    int16_t speed;   //转速rpm
    int16_t torque;   //实际转矩电流
    uint8_t temp;   //电机温度
}Motor_Feedback_t;

extern Motor_Feedback_t motor_feedback[8]={0};


extern float target_yaw_deg;  //视觉目标yaw角度
extern float target_pitch_deg;  //视觉目标pitch角度


#endif //BMI088_DEMO_GIMBAL_H