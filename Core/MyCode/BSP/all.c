#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "bsp_dr16.h"
#include "can.h"
#include "Chassis_motor.h"
#include "portable.h"
#include "stm32f4xx_hal_can.h"
#include "usbd_cdc_if.h"
//
// Created by zhouz on 2026/3/15.
/////**************以下是底盘的发送函数************************/////
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

////*********************以下是云台的驱动函数************************/
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


/******************以下是usb接收的回调函数**********************/
void USB_Vision_ReceiveCallback(uint8_t *Buf,uint32_t Len)
{
    //Len为USB接收到的数据长度

    uint8_t *rx_data=(uint8_t*)pvPortMalloc(Len);
    //从Freertos堆里面声情了一块Len字节大小的内存，用来存放USB接收到的数据

    if (rx_data!=NULL)
    {
        memcpy(rx_data,Buf,Len);//memcpy是一个复制函数
        osMessageQueuePut(VisionDataQueueHandle, &rx_data, 0, 0);
    }
}

///********************以下是usb发送函数********************//////
void USB_Vision_SendEuler(float pitch,float roll,float yaw)
{
    char tx_buf[64]={0};  //发送的数据

    snprintf(tx_buf,sizeof(tx_buf),"pitch:%.2f,roll:%.2f,yaw:%.2f\r\n",pitch,roll,yaw);

    uint16_t len=strlen(tx_buf);
    CDC_Transmit_FS((uint8_t*)tx_buf,len);//这里调用了CDC_Transmit函数，可以直接把对应的数据发送给视觉组
}


/////*********************/* 私有函数：向bmi088加速度计写入单个寄存器 */***************/
void BMI088_Acc_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    tx_buf[0] = (reg & BMI088_SPI_WRITE_CMD); // 组合写命令
    tx_buf[1] = data;

    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET); // 拉低CS，选中加速度计
    HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx_buf, 2, 10);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET); // 拉高CS，结束传输
    osDelay(1); // 短暂延时，确保时序稳定
}


///////*******************向bmi088陀螺仪写寄存器函数（之前缺失的核心实现） ***************/
void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    // 组合写命令：寄存器地址 & 写命令位（BMI088_SPI_WRITE_CMD 是你定义的写掩码）
    tx_buf[0] = (reg & BMI088_SPI_WRITE_CMD);
    tx_buf[1] = data;

    // 拉低陀螺仪CS引脚，选中设备
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
    // 通过SPI发送写命令和数据
    HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx_buf, 2, 1); // 超时时间10ms
    // 拉高CS引脚，结束传输
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    osDelay(1); // 短暂延时保证时序稳定
}



////////////* 私有函数：从加速度计读取多个寄存器（突发读取模式） *////////////
void BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_cmd = reg | BMI088_SPI_READ_CMD; // 组合读命令

    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(BMI088_SPI_HANDLE, &tx_cmd, 1, 1);
    // 注意：根据数据手册，加速度计读操作时，第一个接收字节是“哑元”（dummy），需要丢弃
    uint8_t dummy;

    HAL_SPI_Receive(BMI088_SPI_HANDLE, &dummy, 1, 1);
    // 然后连续读取len个字节的数据

    HAL_SPI_Receive(BMI088_SPI_HANDLE, data, len, 5);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
}


///////////* 陀螺仪的读写函数实现类似，但注意其读操作不需要丢弃第一个哑元字节 *////////////
void BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_cmd = reg | BMI088_SPI_READ_CMD;

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(BMI088_SPI_HANDLE, &tx_cmd, 1, 1);

    // 陀螺仪直接读取有效数据

    HAL_SPI_Receive(BMI088_SPI_HANDLE, data, len, 5);

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}


/////////////*****
////************************ 中断函数-----把得到的数据存放到rc_remoto ************************/
// 入参：pData→DMA接收的18字节原始数据缓冲区
void DR16_Parse(uint8_t *pData)
{
    if(pData == NULL) return;  // 空指针保护，防止程序崩溃

    memset(&rc_remote, 0, sizeof(RC_Remote_t));  // 清空数据，防止脏数据干扰

    /************************ 解析11bit摇杆通道（手册核心位域偏移） ************************/
    // CH0：偏移0bit，长度11bit → pData[0]全8bit + pData[1]低3bit | &0x07FF(11bit掩码，过滤高位)
    rc_remote.ch0 = ((uint16_t)pData[0] | ((uint16_t)pData[1] << 8)) & 0x07FF;
    // CH1：偏移11bit，长度11bit → pData[1]高5bit + pData[2]全8bit
    rc_remote.ch1 = (((uint16_t)pData[1] >> 3) | ((uint16_t)pData[2] << 5)) & 0x07FF;
    // CH2：偏移22bit，长度11bit → pData[2]高2bit + pData[3]全8bit + pData[4]低1bit
    rc_remote.ch2 = (((uint16_t)pData[2] >> 6) | ((uint16_t)pData[3] << 2) | ((uint16_t)pData[4] << 10)) & 0x07FF;
    // CH3：偏移33bit，长度11bit → pData[4]高7bit + pData[5]低4bit

    rc_remote.ch3 = (((uint16_t)pData[4] >> 1) | ((uint16_t)pData[5] << 7)) & 0x07FF;

    /************************ 解析2bit开关S1/S2（手册偏移44/46bit） ************************/
    // S1/S2均在pData[5]的高4bit，手册定义：1=上、2=下、3=中
    rc_remote.s2 = ((pData[5] >> 4) & 0x0C) >> 2;  // 取pData[5] bit6-bit7，保留高两位
    rc_remote.s1 = ((pData[5] >> 4) & 0x03);       // 取pData[5] bit4-bit5

    rc_remote.s1 =((rc_remote.s1==0)?RC_SW_MID :rc_remote.s1);
    rc_remote.s2 =((rc_remote.s2==0)?RC_SW_MID :rc_remote.s2);
}

