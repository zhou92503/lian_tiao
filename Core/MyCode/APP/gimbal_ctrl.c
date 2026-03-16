#include "gimbal_ctrl.h"
#include "motor.h"


Motor_Feedback_t motor_feedback[8]={0};
//用来存放电机的各个参数


float target_yaw_deg=0.0f;  //视觉目标Yaw角度（来自USB)
float target_pitch_deg=0.0f;  //视觉目标Pitch角度

#define MOTOR_ANGLE_TO_DEG(angle)  ((float)angle/8192.0f*360.0f)



///************************云台pid解算函数******************///////////
void Gimbal_PID_Control(void)
{

    // 1. 获取电机当前状态（角度转度数，转速直接用）
    float yaw_current_deg = MOTOR_ANGLE_TO_DEG(motor_feedback[YAW_MOTOR_ID].angle);
    float yaw_current_speed = motor_feedback[YAW_MOTOR_ID].speed;
    float pitch_current_deg = MOTOR_ANGLE_TO_DEG(motor_feedback[PITCH_MOTOR_ID].angle);//获取到当前电机的转动的角度，并映射到360.0f
    float pitch_current_speed = motor_feedback[PITCH_MOTOR_ID].speed;//获取到当前电机的转速（rpm)

    // 2. Yaw轴双环PID
    // 位置环：目标角度 → 速度给定
    float yaw_speed_target = PID_Calc(&pid_yaw_pos, target_yaw_deg, yaw_current_deg);
    // 速度环：速度给定 → 电流输出
    int16_t yaw_current = (int16_t)PID_Calc(&pid_yaw_speed, yaw_speed_target, yaw_current_speed);

    // 3. Pitch轴双环PID
    // 位置环：目标角度 → 速度给定
    float pitch_speed_target = PID_Calc(&pid_pitch_pos, target_pitch_deg, pitch_current_deg);
    // 速度环：速度给定 → 电流输出
    int16_t pitch_current = (int16_t)PID_Calc(&pid_pitch_speed, pitch_speed_target, pitch_current_speed);

    // 4. CAN发送电流指令
    CAN_Motor_SetVoltage(yaw_current, pitch_current);
}



/////////****************************云台pid的参数***********************//////////
void Gimbal_Init(void)
{
    // Yaw轴位置环：kp=8, ki=0, kd=0.1, 输出限幅±500rpm, 积分限幅0
    PID_Init(&pid_yaw_pos, 8.0f, 0.0f, 0.1f, 500.0f, -500.0f, 0.0f, 0.0f);
    // Yaw轴速度环：kp=20, ki=0.5, kd=0, 输出限幅±16384（最大电流）, 积分限幅±1000
    PID_Init(&pid_yaw_speed, 20.0f, 0.5f, 0.0f, 16384.0f, -16384.0f, 1000.0f, -1000.0f);

    // Pitch轴位置环：kp=10, ki=0, kd=0.2, 输出限幅±300rpm, 积分限幅0
    PID_Init(&pid_pitch_pos, 10.0f, 0.0f, 0.2f, 300.0f, -300.0f, 0.0f, 0.0f);
    // Pitch轴速度环：kp=25, ki=0.8, kd=0, 输出限幅±16384, 积分限幅±1000
    PID_Init(&pid_pitch_speed, 25.0f, 0.8f, 0.0f, 16384.0f, -16384.0f, 1000.0f, -1000.0f);
}