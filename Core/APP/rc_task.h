//
// Created by zhouz on 2026/2/26.
//

#ifndef BMI088_DEMO_RC_TASK_H
#define BMI088_DEMO_RC_TASK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp_dr16.h"

typedef struct {
    float forward;   ///前进/后退 （-500~500）
    float left;     ///左移/右移（-500~500）
    float rotete;   ///原地旋转 （-500~500）
    float speed;    ///旋转倍率  （0.2、0.5、1.0）
}Chassis_RC_Cmd_t;

// 新增：硬件参数定义（根据你的实际值修改！）
#define PI              3.1415926f
#define GEAR_RATIO      19.0f        // M3508减速比
#define WHEEL_RADIUS    0.05f        // 轮子半径（米）
#define CHASSIS_RADIUS  0.735f         // 底盘中心到轮子的距离（米）
#define MAX_MOTOR_RPM   2000.0f      // 电机最大转速（新手推荐2000rpm）
#define MAX_CURR        16384        // C620最大电流（-16384~16384）
#define RPM_TO_CURR     (MAX_CURR / MAX_MOTOR_RPM) // rpm转电流的系数



// 新增：底盘结构体（存放角度等关键参数）
typedef struct {
    float angle;          // 底盘当前角度（弧度，由IMU/云台yaw提供）
    float Radius;         // 底盘轮距半径
} Chassis_t;
extern Chassis_t chassis;



extern  Chassis_RC_Cmd_t chassis_rc_cmd;
extern int16_t motor_target[4];
extern   SemaphoreHandle_t xDR16DataSemaphore ;
#endif //BMI088_DEMO_RC_TASK_H