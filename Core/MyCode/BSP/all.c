#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "can.h"
#include "dr16.h"


#include "all.h"
#include "portable.h"
#include "usbd_cdc_if.h"

UART_HandleTypeDef *dr16_huart=NULL;

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


///////*******************以下是对huart5串口初始化的函数****************/////
void DR16_Init(UART_HandleTypeDef *huart)
{

    dr16_huart=huart;
    ///新增：清除空闲中断标志（防止初始化时触发的无效中断）
    __HAL_UART_CLEAR_IDLEFLAG(dr16_huart);
    //开启DMA接收：初始化缓冲区0，接收18字节
    HAL_UART_Receive_DMA(huart, dr16_rx_buf[0], RC_FRAME_LEN);
    ////开启UART空闲中断（核心：一帧接收完成后就出发中断，进行解析）
    __HAL_UART_ENABLE_IT(dr16_huart, UART_IT_IDLE);
}

