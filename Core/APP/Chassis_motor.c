#include "FreeRTOS.h"
#include "task.h"
#include  "bsp_dr16.h"
#include "can.h"
#include "Chassis_motor.h"

#include <sys/types.h>

#include "cmsis_os2.h"
#include "rc_task.h"


extern int16_t motor_target[4];

static void Chassis_CAN_Send_M3508(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8]={0};
    uint32_t tx_mailbox;

    //配置CAN发送头（C620电调标准配置）
    tx_header.StdId=C620_CTRL_CAN_ID;  //控制指令固定ID : 0x200
    tx_header.RTR=CAN_RTR_DATA;//数据帧
    tx_header.IDE=CAN_ID_STD;//标准ID
    tx_header.DLC=8;  //数据长度8字节
    tx_header.TransmitGlobalTime=DISABLE;


    //*******************2.限制目标电流（防M3508过流损坏）
    motor1 = (motor1>C620_CURR_MAX) ? C620_CURR_MAX : (motor1 < C620_CURR_MIN) ? C620_CURR_MIN : motor1;
    motor2 = (motor2>C620_CURR_MAX) ? C620_CURR_MAX : (motor2 < C620_CURR_MIN) ? C620_CURR_MIN : motor2;
    motor3 = (motor3>C620_CURR_MAX) ? C620_CURR_MAX : (motor3 < C620_CURR_MIN) ? C620_CURR_MIN : motor3;
    motor4 = (motor4>C620_CURR_MAX) ? C620_CURR_MAX : (motor4 < C620_CURR_MIN) ? C620_CURR_MIN : motor4;

    /********************3. 封装八字节数据（官方字节序：高字节在前）*********************/

   //ID1电机（M3508_1) :字节0=高八位，字节1=低八位
    tx_data[0] = (motor1>>8) & 0xFF;
    tx_data[1] = (motor1) & 0xFF;
    //ID2电机（M3508_2) ：字节2=高八位，字节3=低八位
    tx_data[2] = (motor2>>8) & 0xFF;
    tx_data[3] = (motor2) & 0xFF;
    //ID3电机（M3508_3) ：字节4=高八位，字节5=低八位
    tx_data[4] = (motor3>>8) & 0xFF;
    tx_data[5] = (motor3) & 0xFF;
    //ID4电机（M3508_4) ：字节6=高八位，字节7=低八位
    tx_data[6] = (motor4>>8) & 0xFF;
    tx_data[7] = (motor4) & 0xFF;

    HAL_CAN_AddTxMessage(CHASSIS_CAN,&tx_header,tx_data,&tx_mailbox);
}



void Start_Chassis_CAN_Task(void *argument)
{
    (void)argument;

    HAL_CAN_Start(CHASSIS_CAN);  //开启can1的通道
    HAL_CAN_ActivateNotification(CHASSIS_CAN,CAN_IT_TX_MAILBOX_EMPTY);


    for(;;)
    {
        Chassis_CAN_Send_M3508(motor_target[0],motor_target[1],motor_target[2],motor_target[3]);


        osDelay(5);
    }
}

