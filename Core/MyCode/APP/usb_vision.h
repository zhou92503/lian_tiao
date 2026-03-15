//
// Created by zhouz on 2026/2/22.
//

#ifndef BMI088_DEMO_USB_VISION_H
#define BMI088_DEMO_USB_VISION_H
#include <stdint.h>

#include "main.h"       // 先包含 main.h（里面会包含 stm32f4xx_hal.h 等）
#include "FreeRTOS.h"   // 先包含 FreeRTOS 核心头
#include "task.h"       // 再包含 task.h
#include "cmsis_os2.h"  // 最后包含 CMSIS-RTOS2 封装层

extern  float target_pitch ;
extern  float target_yaw ;
extern  float target_roll;



void USB_Vision_ReceiveCallback(uint8_t *Buf,uint32_t Len);
void USB_Vision_ParseEuler(uint8_t *buf,uint16_t len);
void Start_Vision_Process_Task(void *argument);

void USB_Vision_SendEuler(float pitch,float roll,float yaw);

#endif //BMI088_DEMO_USB_VISION_H