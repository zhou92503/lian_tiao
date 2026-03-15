
#include "rc_task.h"

#include <stdlib.h>
#include <tgmath.h>

#include "../BSP/bsp_dr16.h"
#include "../Module/Chassis_motor.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "can.h"

extern osThreadId_t RC_TaskHandle;

Chassis_t chassis = {0.0f, CHASSIS_RADIUS};

SemaphoreHandle_t xDR16DataSemaphore = NULL;
//FreeRTOS中信号量，用来唯一识别一个信号

Motor_t motor[4];

static int16_t RC2RPM(uint16_t rc_val,float max_rpm)
{
      int32_t rc_diff = 0;
      float target_rpm = 0.0f;

     //1.计算摇杆相对中点的偏差值
     rc_diff=rc_val - RC_CH_MID;

    //2.死区判断：中点附近微笑偏差，直接返回零电流（防抖）
    if (abs(rc_diff)<=RC_DEAD_ZONE)
    {
        return C620_CURR_MID;
    }

    // 3. 线性映射：摇杆偏差 -> 目标rpm（-max_rpm~max_rpm）
    target_rpm = (float)((rc_diff * max_rpm) / RC_STROKE);

    if (target_rpm>max_rpm)  target_rpm = max_rpm;
    else if (target_rpm < -max_rpm) target_rpm = -max_rpm;

    return target_rpm;
}




// 新增：角度归一化（-π~π），避免角度溢出
static float normalize_angle(float angle)
{
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}




static void RC_Calc_MotorSpeed(void)
{
    float forward_rpm = 0.0f, left_rpm = 0.0f, rotate_rpm = 0.0f;
    float speed_rate = 1.0f;
    float speed_cal[4] = {0.0f};
    float sin_ang, cos_ang;
    /************************ 1.左摇杆控制底盘前后左右************************/

    //左摇杆前后：前进/后退（CH3)----------向前为正
    forward_rpm = -RC2RPM(rc_remote.ch3,MAX_MOTOR_RPM);
    ///右摇杆左右 :左移/右移

    ////------------------------------向右为正
    left_rpm=RC2RPM(rc_remote.ch2,MAX_MOTOR_RPM);
    //左摇杆前后：前进/后退（CH3)
    rotate_rpm = RC2RPM(rc_remote.ch0,MAX_MOTOR_RPM)/2;
    ///右摇杆左右 :左移/右移


    //*************************2.右摇杆控制云台YAW/PITCH************************/
    switch (rc_remote.s2)
    {
        case RC_SW_UP:    speed_rate=1.0f;   break;
        case RC_SW_MID:   speed_rate=0.5f;   break;
        case RC_SW_DOWN:  speed_rate=0.2f;   break;
        default:          speed_rate=0.0f;   break;
    }
}


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




// 新增：更新底盘角度（需在IMU/云台任务中调用，传入云台yaw或IMU角度）
void Chassis_Update_Angle(float new_angle)
{
    chassis.angle = new_angle;
}


 void Start_RC_Parse_Task(void *argument)
{

    (void)argument;

    xDR16DataSemaphore =xSemaphoreCreateBinary();
   ///创建一个“空的信号量”，信号量的初始值为0

    configASSERT(xDR16DataSemaphore !=NULL);//创建失败则断言，方便调试


     //DR16 初始化(UART5句柄，CubeMx生成的&huart5)
    DR16_Init(&huart5);

    for(;;)
    {
        if (xSemaphoreTake(xDR16DataSemaphore,osWaitForever)==pdTRUE)
      ///

        RC_Calc_MotorSpeed();

        //延迟10ms,和DR16刷新率（14ms)匹配，降低CPU占用
        osDelay(10);
    }
}