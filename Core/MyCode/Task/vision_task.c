#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "main.h"
#include "portable.h"


float target_pitch =0.0f;
float target_yaw =0.0f;
float target_roll =0.0f;


/////*****************************提取视觉组发送字节数中的姿态角函数****************************/////
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




////// ====================== 3. 视觉数据处理Task（FreeRTOS Task） ======================
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