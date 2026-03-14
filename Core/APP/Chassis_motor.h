//
// Created by zhouz on 2026/2/27.
//

#ifndef BMI088_DEMO_CHASSIS_MOTOR_H
#define BMI088_DEMO_CHASSIS_MOTOR_H


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


#endif //BMI088_DEMO_CHASSIS_MOTOR_H