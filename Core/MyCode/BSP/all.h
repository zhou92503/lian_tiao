//
// Created by zhouz on 2026/3/15.
//

#ifndef BMI088_DEMO_ALL_H
#define BMI088_DEMO_ALL_H

#include "stm32f4xx_hal.h"

void DR16_Init(UART_HandleTypeDef *huart);

void USB_Vision_ReceiveCallback(uint8_t *Buf,uint32_t Len);
void USB_Vision_SendEuler(float pitch,float roll,float yaw);



#endif //BMI088_DEMO_ALL_H