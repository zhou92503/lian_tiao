


#include "chassis_ctrl.h"
#include "cmsis_os2.h"
#include "motor.h"

////**************************以下是底盘电机发送任务函数*********************//////

void Start_Chassis_CAN_Task(void *argument)
{
    (void)argument;

    HAL_CAN_Start(CHASSIS_CAN);  //开启can1的通道
    HAL_CAN_ActivateNotification(CHASSIS_CAN,CAN_IT_TX_MAILBOX_EMPTY);


    for(;;)
    {

        Chassis_CAN_Send_M3508(motor[0].current,motor[1].current,motor[2].current,motor[3].current);

        osDelay(5);
    }
}
