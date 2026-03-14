/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for UART_Print_Task */
osThreadId_t UART_Print_TaskHandle;
const osThreadAttr_t UART_Print_Task_attributes = {
  .name = "UART_Print_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for BMI088_Read_Tas */
osThreadId_t BMI088_Read_TasHandle;
const osThreadAttr_t BMI088_Read_Tas_attributes = {
  .name = "BMI088_Read_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VisionProcessTa */
osThreadId_t VisionProcessTaHandle;
const osThreadAttr_t VisionProcessTa_attributes = {
  .name = "VisionProcessTa",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for CAN_Motor_Task */
osThreadId_t CAN_Motor_TaskHandle;
const osThreadAttr_t CAN_Motor_Task_attributes = {
  .name = "CAN_Motor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RC_Parse_Task */
osThreadId_t RC_Parse_TaskHandle;
const osThreadAttr_t RC_Parse_Task_attributes = {
  .name = "RC_Parse_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Chassis_CAN_Tas */
osThreadId_t Chassis_CAN_TasHandle;
const osThreadAttr_t Chassis_CAN_Tas_attributes = {
  .name = "Chassis_CAN_Tas",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};



/* Definitions for BMI088DataQueue */
osMessageQueueId_t BMI088DataQueueHandle;
const osMessageQueueAttr_t BMI088DataQueue_attributes = {
  .name = "BMI088DataQueue"
};
/* Definitions for VisionDataQueue */
osMessageQueueId_t VisionDataQueueHandle;
const osMessageQueueAttr_t VisionDataQueue_attributes = {
  .name = "VisionDataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_UART_Print_TaskTask(void *argument);
void Start_BMI088_Read_Task_Task(void *argument);
void Start_Vision_Process_Task(void *argument);
void Start_CAN_Motor_Task(void *argument);
void Start_RC_Parse_Task(void *argument);
void Start_Chassis_CAN_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of BMI088DataQueue */
  BMI088DataQueueHandle = osMessageQueueNew (8, 4, &BMI088DataQueue_attributes);

  /* creation of VisionDataQueue */
  VisionDataQueueHandle = osMessageQueueNew (16, 4, &VisionDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Create the thread(s) */
  /* creation of UART_Print_Task */
  UART_Print_TaskHandle = osThreadNew(Start_UART_Print_TaskTask, NULL, &UART_Print_Task_attributes);

  /* creation of BMI088_Read_Tas */
  BMI088_Read_TasHandle = osThreadNew(Start_BMI088_Read_Task_Task, NULL, &BMI088_Read_Tas_attributes);

  /* creation of VisionProcessTa */
  VisionProcessTaHandle = osThreadNew(Start_Vision_Process_Task, NULL, &VisionProcessTa_attributes);

  /* creation of CAN_Motor_Task */
  CAN_Motor_TaskHandle = osThreadNew(Start_CAN_Motor_Task, NULL, &CAN_Motor_Task_attributes);

  /* creation of RC_Parse_Task */
  RC_Parse_TaskHandle = osThreadNew(Start_RC_Parse_Task, NULL, &RC_Parse_Task_attributes);

  /* creation of Chassis_CAN_Tas */
  Chassis_CAN_TasHandle = osThreadNew(Start_Chassis_CAN_Task, NULL, &Chassis_CAN_Tas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_UART_Print_TaskTask */
/**
  * @brief  Function implementing the UART_Print_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_UART_Print_TaskTask */
__weak void Start_UART_Print_TaskTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Start_UART_Print_TaskTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_UART_Print_TaskTask */
}

/* USER CODE BEGIN Header_Start_BMI088_Read_Task_Task */
/**
* @brief Function implementing the BMI088_Read_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_BMI088_Read_Task_Task */
__weak void Start_BMI088_Read_Task_Task(void *argument)
{
  /* USER CODE BEGIN Start_BMI088_Read_Task_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_BMI088_Read_Task_Task */
}

/* USER CODE BEGIN Header_Start_Vision_Process_Task */
/**
* @brief Function implementing the VisionProcessTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Vision_Process_Task */
__weak void Start_Vision_Process_Task(void *argument)
{
  /* USER CODE BEGIN Start_Vision_Process_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Vision_Process_Task */
}

/* USER CODE BEGIN Header_Start_CAN_Motor_Task */
/**
* @brief Function implementing the CAN_Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CAN_Motor_Task */
__weak void Start_CAN_Motor_Task(void *argument)
{
  /* USER CODE BEGIN Start_CAN_Motor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_CAN_Motor_Task */
}

/* USER CODE BEGIN Header_Start_RC_Parse_Task */
/**
* @brief Function implementing the RC_Parse_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RC_Parse_Task */
__weak void Start_RC_Parse_Task(void *argument)
{
  /* USER CODE BEGIN Start_RC_Parse_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_RC_Parse_Task */
}

/* USER CODE BEGIN Header_Start_Chassis_CAN_Task */
/**
* @brief Function implementing the Chassis_CAN_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Chassis_CAN_Task */
__weak void Start_Chassis_CAN_Task(void *argument)
{
  /* USER CODE BEGIN Start_Chassis_CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Chassis_CAN_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

