//
// Created by zhouz on 2026/3/15.
//

#ifndef BMI088_DEMO_MOTOR_H
#define BMI088_DEMO_MOTOR_H
#include <stdint.h>

#include "stm32f4xx_hal_can.h"


void Chassis_CAN_Send_M3508(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void CAN_Parse_Motor_Feedback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void CAN_Motor_SetVoltage(int16_t yaw_current,int16_t pitch_current);

#define  CHASSIS_CAN    &hcan1

//C620电调标准CAN   ID：4个电机对应0x201~0x204
#define     CAN_ID_MOTOR1    0x201
#define     CAN_ID_MOTOR2    0x202
#define     CAN_ID_MOTOR3    0x203
#define     CAN_ID_MOTOR4    0x204


#define C620_CTRL_CAN_ID   0x200
// C620电调目标电流范围（官方标准）：-16384 ~ +16384（对应电机正反转）
#define C620_CURR_MAX       16384
#define C620_CURR_MIN       -16384
#define C260_CURR_MID       0



 ///***********************一下数据是视觉组发来的目标角度（姿态角）*****************///////
extern float target_pitch =0.0f;
extern float target_yaw =0.0f;
extern float target_roll =0.0f;





#endif //BMI088_DEMO_MOTOR_H