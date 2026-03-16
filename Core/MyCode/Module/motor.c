#include "motor.h"

#include <stdint.h>

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "gimbal_ctrl.h"

////*************************以下是底盘电机的can发送函数**************************/////////
void Chassis_CAN_Send_M3508(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
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


////*********************以下是云台电机的发送送函数************************/
void CAN_Motor_SetVoltage(int16_t yaw_current,int16_t pitch_current)
{
    //yaw/pitch_current的值分别对应两个电机的给定电流值，电流给定范围-16384~16384；
    ///以上是通过电压来控制对应的转速
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8]={0};
    uint32_t tx_mailbox;

    tx_header.StdId=0x1FF;
    tx_header.RTR=CAN_RTR_DATA;
    tx_header.IDE=CAN_ID_STD;
    tx_header.DLC=8;

    tx_data[0] = (yaw_current >> 8) & 0xFF;
    tx_data[1] = yaw_current & 0xFF;
    tx_data[2] = (pitch_current >> 8) & 0xFF;
    tx_data[3] = pitch_current & 0xFF;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}


/////*****************以下是云台电机中断接收函数*********************///////
void CAN_Parse_Motor_Feedback(CAN_RxHeaderTypeDef *rx_header,uint8_t *rx_data)
{
    //如果当前接收到的can的标识符是0x204-0x20B
    if (rx_header->StdId>=0x204&&rx_header->StdId<=0x20B)
    {
        uint8_t motor_id=rx_header->StdId-0x204;

        motor_feedback[motor_id].angle=(rx_data[0]<<8)|rx_data[1];

        motor_feedback[motor_id].speed=(rx_data[2]<<8)|rx_data[3];

        motor_feedback[motor_id].torque=(rx_data[4]<<8)|rx_data[5];

        motor_feedback[motor_id].temp=rx_data[6];
    }
}