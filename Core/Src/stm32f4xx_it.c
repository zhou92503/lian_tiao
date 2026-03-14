/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_dr16.h"
#include "rc_task.h"
#include "task.h"
#include "FreeRTOS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)//只要触发了中断请求（IRQ)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //只在第一次进入中断时创建这个变量，后续进入中断复用这个变量（省内存，避免重复创建)
  //xHigherPriorityTaskWoken--字面意思“高优先级任务是否被唤醒”，初始值pdFALSE


//xHigherPriorityTaskWoken变量:可以理解为”高优先级任务已经完成，可以开始执行新的高优先级任务了“
  if(dr16_huart != NULL && dr16_huart->Instance == UART5)
  {
    // 检测是否为空闲中断（一帧18字节接收完成的标志）
    if(__HAL_UART_GET_FLAG(dr16_huart, UART_FLAG_IDLE) != RESET)
    {
      __HAL_UART_CLEAR_IDLEFLAG(dr16_huart);  // 清除空闲中断标志，避免重复触发
      HAL_UART_DMAStop(dr16_huart);           // 停止DMA接收，准备解析

      // 计算实际接收的字节数：总长度18 - DMA剩余未接收字节数
      uint16_t recv_len = RC_FRAME_LEN - __HAL_DMA_GET_COUNTER(dr16_huart->hdmarx);
      if(recv_len == RC_FRAME_LEN)  // 仅处理**完整的18字节帧**，保证解析正确
      {
        // DMA双缓冲切换：解析当前缓冲区，重启另一缓冲区接收下一包
        if(dr16_huart->hdmarx->Instance->M0AR == (uint32_t)dr16_rx_buf[0])
        {
          DR16_Parse(dr16_rx_buf[0]);  // 解析缓冲区0的18字节数据
          HAL_UART_Receive_DMA(dr16_huart, dr16_rx_buf[1], RC_FRAME_LEN);  // 重启缓冲区1
        }
        else
        {
          DR16_Parse(dr16_rx_buf[1]);  // 解析缓冲区1的18字节数据
          HAL_UART_Receive_DMA(dr16_huart, dr16_rx_buf[0], RC_FRAME_LEN);  // 重启缓冲区0
        }

        if (xDR16DataSemaphore != NULL)//先检查“通知开关（信号量）”是否存在
          //如果之前的创建信号量失败（xDR16DataSemaphore）则创建失败，返回NULL
        {
          xSemaphoreGiveFromISR(xDR16DataSemaphore, &xHigherPriorityTaskWoken);
          //把 “唤醒标记” 的地址传进去 —— 如果释放信号量后，RC 任务的优先级比当前 CPU 正在执行的任务高，
          //这个函数会把xHigherPriorityTaskWoken改成pdTRUE（1）
          //给xDR16DataSemaphore信号量'发信号‘，并且如果

          portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //唤醒RC任务
        }

      }
      else  // 接收不完整（丢包），重启当前缓冲区，重新接收
      {
        HAL_UART_Receive_DMA(dr16_huart, (uint8_t*)dr16_huart->hdmarx->Instance->M0AR, RC_FRAME_LEN);
      }
    }
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
