//
// Created by zhouz on 2026/2/23.
//
#include "pid.h"
#include <main.h>

PID_HandleTypeDef pid_yaw_pos;//yaw_角度环的pid
PID_HandleTypeDef pid_yaw_speed;  //yaw_速度环pid
PID_HandleTypeDef pid_pitch_pos; //pitch_角度环的pid
PID_HandleTypeDef pid_pitch_speed;//pitch_速度环pid

//这个函数表示把对应的pid放到这个PID_HandleTypeDef结构体中
void PID_Init(PID_HandleTypeDef *pid,float kp,float ki,float kd,float out_max,float out_min,float i_max,float i_min)
{

    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;

    pid->out_max=out_max;
    pid->out_min=out_min;
    pid->i_min=i_min;

    pid->target=0.0f;
    pid->current=0.0f;
    pid->err=0.0f;
    pid->last_err=0.0f;
    pid->err_sum=0.0f;
}

//PID位置计算
float PID_Calc(PID_HandleTypeDef *pid,float target,float current)
{
    pid->target=target;
    pid->current=current;

    pid->err=pid->target-pid->current;
    pid->err_sum+=pid->err;

    if (pid->err_sum > pid->i_max)  pid->err_sum=pid->i_max;
    if (pid->err_sum < pid->i_min)  pid->err_sum=pid->i_min;

    //pid计算
    float output=pid->kp*pid->err+pid->ki*pid->err_sum+pid->kd*(pid->err-pid->last_err);

    if (output>pid->out_max)  output=pid->out_max;
    if (output<pid->out_min)  output=pid->out_min;

    pid->last_err = pid->err;

    return  output;
}

void PID_Clear(PID_HandleTypeDef *pid)

{
     pid->err=0.0f;
     pid->err_sum=0.0f;
     pid->last_err=0.0f;
}
