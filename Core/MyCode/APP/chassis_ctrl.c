
#include <stdint.h>
#include <stdlib.h>
#include <tgmath.h>

#include "APP/chassis_ctrl.h"
#include "cmsis_os2.h"


#include "dr16.h"

extern osThreadId_t RC_TaskHandle;

Chassis_t chassis = {0.0f, CHASSIS_RADIUS};


Motor_t motor[4];




///*****************************底盘解算函数********************************//////
void chassis_move(float target_speed, float target_dir, float target_omega)
{
    /********直接计算方法********/
    float speed_cal[4];

    float rad = chassis.angle*PI / 180.0f;///将angle(°）转变成rad


    float sin_ang = sin(rad);
    float cos_ang = cos(rad);

    float speed_X = target_speed * cos(target_dir);
    float speed_Y = target_speed * sin(target_dir);

    speed_cal[0] = ((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[1] = ((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[2] = ((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[3] = ((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);

    //限幅RPM
    int speed_out[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++)
    {
        speed_out[i] = (int)speed_cal[i];

        if (speed_out[i]>2000) speed_out[i]= 2000;
        if (speed_out[i]<-2000) speed_out[i]= -2000;

        motor[i].target = speed_out[i];    ///将我计算出得到的速度（rpm）赋给电机目标值
        motor[i].current=PID_Calc(&motor[i].pid,motor[i].current,motor[i].target);
    }
}