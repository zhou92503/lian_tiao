


#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "cmsis_os.h"
#include "usart.h"

void Start_UART_Print_TaskTask(void *argument)
{
    BMI088_Data_t *p_print_data =NULL;
    char print_buf[128]={0};

    for (;;)
    {
        if (osMessageQueueGet(BMI088DataQueueHandle,&p_print_data,NULL,osWaitForever)==osOK)
        {
            snprintf(print_buf, sizeof(print_buf),"pitch:%.2f,yaw:%.2f,roll:%.2f\r\n",
                          p_print_data->pitch, p_print_data->yaw, p_print_data->roll);


            HAL_UART_Transmit(&huart1,(uint8_t *)print_buf,strlen(print_buf),1000);

            vPortFree(p_print_data);
            p_print_data=NULL;
        }
        osDelay(10);
    }
}
