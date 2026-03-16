

#include <stdint.h>

#include "all.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "cmsis_os.h"



#include "bmi088.h"
#include "chassis_ctrl.h"
#include "filtering.h"



///***********************以下是关于bmi088读取的任务函数******************///////
void Start_BMI088_Read_Task_Task(void *argument)
{

    uint8_t init_err = BMI088_Init();
    if (init_err != 0)
    {

        osThreadTerminate(osThreadGetId());   //osThreadGetId() 获取当前运行任务的 ID，
        //osThreadTerminate 终止这个任务（相当于 “杀死” 当前任务）；
        return;

    }
    BMI088_Data_t angle_data;

    float ax,ay,az,gx,gy,gz;
    // 作用：给BMI088发送初始化命令（解锁、复位、配置量程/采样率），让传感器进入工作状态
    // &hspi1：告诉初始化函数“用SPI1和传感器通信”

    for (;;)
    {

        BMI088_Data_t *p_data=(BMI088_Data_t*)pvPortMalloc(sizeof(BMI088_Data_t));
        // 申请内存，用于存储传感器数据
        //p_data:指向新申请的内存地址

        if (p_data==NULL)
        {
            osDelay(10);
            continue;
        }

        BMI088_ReadAcceleration(&ax,&ay,&az);//分别得到各个方向的角速度（单位rad/s)
        BMI088_ReadGyroscope(&gx,&gy,&gz);  //分别得到各个方向的加速度（单位g)

        MahonyAHRSupdate(gx,gy,gz,ax,ay,az);

        // ③ 把姿态角赋值给结构体
        p_data->pitch = Pitch;
        p_data->roll  = Roll;
        p_data->yaw   = Yaw;

        chassis.angle=Yaw;////把当前的姿态角度赋给底盘结构体

        USB_Vision_SendEuler(p_data->pitch, p_data->roll, p_data->yaw);
        osMessageQueuePut(BMI088DataQueueHandle, &p_data, 0, 0);

        osDelay(10);
    }
}