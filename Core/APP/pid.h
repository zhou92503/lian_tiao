//
// Created by zhouz on 2026/2/23.
//

#ifndef BMI088_DEMO_PID_H
#define BMI088_DEMO_PID_H


typedef  struct
{
      float target;
      float current;

      float kp;
      float ki;
      float kd;

      float err;
      float last_err;
      float err_sum;

      //输出限幅
      float out_max;
      float out_min;

      ///积分限幅
      float i_max;
      float i_min;
}PID_HandleTypeDef;

extern PID_HandleTypeDef pid_yaw_pos;//Yaw位置环
extern PID_HandleTypeDef pid_yaw_speed;//Yaw速度环
extern PID_HandleTypeDef pid_pitch_pos;//Pitch位置环
extern PID_HandleTypeDef pid_pitch_speed;//Pitch速度环

void PID_Init(PID_HandleTypeDef *pid,float kp,float ki,float kd,float out_max,float out_min,float i_max,float i_min);
float PID_Calc(PID_HandleTypeDef *pid,float target,float current);
void PID_Clear(PID_HandleTypeDef *pid);



#endif //BMI088_DEMO_PID_H