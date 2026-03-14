//
// Created by zhouz on 2026/2/22.
//

#ifndef BMI088_DEMO_CAN_MOTOR_H
#define BMI088_DEMO_CAN_MOTOR_H

#include "main.h"
#include "can.h"
#include "pid.h"

#define YAW_MOTOR_ID 0
#define PITCH_MOTOR_ID 1



typedef struct
{
    uint16_t angle;  //机械角度 0~8191
    int16_t speed;   //转速rpm
    int16_t torque;   //实际转矩电流
    uint8_t temp;   //电机温度
}Motor_Feedback_t;


void CAN_Motor_SetVoltage(int16_t yaw_current,int16_t pitch_current);
void CAN_Parse_Motor_Feedback(CAN_RxHeaderTypeDef *rx_header,uint8_t *rx_data);
void Gimbal_PID_Cotrl(void);
void Gimbal_Init(void);



extern Motor_Feedback_t motor_feedback[8];
extern float target_yaw_deg;  //视觉目标yaw角度
extern float target_pitch_deg;  //视觉目标pitch角度


#endif //BMI088_DEMO_CAN_MOTOR_H