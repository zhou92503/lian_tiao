


#include "usart.h"
#include "chassis_ctrl.h"
#include "all.h"
#include "dr16.h"
#include "FreeRTOSConfig.h"

SemaphoreHandle_t xDR16DataSemaphore = NULL;
//FreeRTOS中信号量，用来唯一识别一个信号

///*****************************以下是dr16解析发送函数********************///////
void Start_RC_Parse_Task(void *argument)
 {

     (void)argument;

     xDR16DataSemaphore =xSemaphoreCreateBinary();
     ///创建一个“空的信号量”，信号量的初始值为0

     configASSERT(xDR16DataSemaphore !=NULL);//创建失败则断言，方便调试


     //DR16 初始化(UART5句柄，CubeMx生成的&huart5)
     DR16_Init(&huart5);

     for(;;)
     {
         if (xSemaphoreTake(xDR16DataSemaphore,osWaitForever)==pdTRUE)


             RC_Calc_MotorSpeed();
         chassis_move(target_speed, target_dir, target_omega);

         //延迟10ms,和DR16刷新率（14ms)匹配，降低CPU占用
         osDelay(10);
     }
 }