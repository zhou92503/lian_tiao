
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "gimbal_ctrl.h"


void Start_CAN_Motor_Task(void *argument)
{

    HAL_CAN_Start(&hcan2);

    Gimbal_Init();

    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    for(;;)
    {

        target_yaw_deg=target_yaw;//这个target_yaw表示视觉组发送过来的yaw角度，然后把这个值赋给target_yaw_deg
        target_pitch_deg=target_pitch;

        Gimbal_PID_Control();

        osDelay(1);
    }
}
