#include "usb_vision.h"
#include  "main.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "can_motor.h"


float target_pitch =0.0f;
float target_yaw =0.0f;
float target_roll =0.0f;

////这是usb虚拟串口给stm32发送的数据，然后就会触发这个回调函数
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


void USB_Vision_ParseEuler(uint8_t *buf,uint16_t len)
{
    //定义字符串缓冲区，用于存储从USB接收到的数据
    char str_buf[64]={0};

    memcpy(str_buf,buf,len>63?63:len);
   //把接收的数据拷贝到字符串缓冲区中

    sscanf(str_buf, "pitch:%f,yaw:%f,roll:%f", &target_pitch, &target_yaw, &target_roll);
    //视觉组必须严格发送标准的格式我才可以进行解析 ，提取里面的三个数据，分别存放到target_pitch，target_yaw，target_roll中

    // 格式2：如果视觉只发数值（如 "15.5,-30.2,2.1\n"），注释上面，打开下面

    // int ret = sscanf(str_buf, "%f,%f,%f", &target_pitch, &target_yaw, &target_roll);
    }

/**
  * @brief  USB_Vision_SendEuler
  * @param  pitch,roll,yaw
  * @retval None
  */
//以下是关于发送数据给视觉组的函数
void USB_Vision_SendEuler(float pitch,float roll,float yaw)
{
    char tx_buf[64]={0};  //发送的数据

    snprintf(tx_buf,sizeof(tx_buf),"pitch:%.2f,roll:%.2f,yaw:%.2f\r\n",pitch,roll,yaw);

    uint16_t len=strlen(tx_buf);
    CDC_Transmit_FS((uint8_t*)tx_buf,len);//这里调用了CDC_Transmit函数，可以直接把对应的数据发送给视觉组
}

// ====================== 3. 视觉数据处理Task（FreeRTOS Task） ======================
void Start_Vision_Process_Task(void *argument)
{

    uint8_t *p_rx_data=NULL;

    for (;;)
    {

        if (osMessageQueueGet(VisionDataQueueHandle, &p_rx_data, NULL, osWaitForever) == osOK)
        {
          //得到队列发来的数据，并且让指针（p_rx_data)指向这个数据

            if (p_rx_data != NULL)
            {
                USB_Vision_ParseEuler(p_rx_data, strlen((char*)p_rx_data));
                vPortFree(p_rx_data);
                p_rx_data = NULL;
            }
        }
        osDelay(1);
    }
}