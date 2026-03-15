#include "can_motor.h"
#include "../APP/usb_vision.h"
#include "../ALG/pid.h"

Motor_Feedback_t motor_feedback[8]={0};
//用来存放电机的各个参数


float target_yaw_deg=0.0f;  //视觉目标Yaw角度（来自USB)
float target_pitch_deg=0.0f;  //视觉目标Pitch角度

#define MOTOR_ANGLE_TO_DEG(angle)  ((float)angle/8192.0f*360.0f)


void CAN_Parse_Motor_Feedback(CAN_RxHeaderTypeDef *rx_header,uint8_t *rx_data)
{
    //如果当前接收到的can的标识符是0x204-0x20B
    if (rx_header->StdId>=0x204&&rx_header->StdId<=0x20B)
    {
        uint8_t motor_id=rx_header->StdId-0x204;

        motor_feedback[motor_id].angle=(rx_data[0]<<8)|rx_data[1];

        motor_feedback[motor_id].speed=(rx_data[2]<<8)|rx_data[3];

        motor_feedback[motor_id].torque=(rx_data[4]<<8)|rx_data[5];

        motor_feedback[motor_id].temp=rx_data[6];
    }
}


void CAN_Motor_SetVoltage(int16_t yaw_current,int16_t pitch_current)
{
    //yaw/pitch_current的值分别对应两个电机的给定电流值，电流给定范围-16384~16384；
    ///以上是通过电压来控制对应的转速
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8]={0};
    uint32_t tx_mailbox;

    tx_header.StdId=0x1FF;
    tx_header.RTR=CAN_RTR_DATA;
    tx_header.IDE=CAN_ID_STD;
    tx_header.DLC=8;

    tx_data[0] = (yaw_current >> 8) & 0xFF;
    tx_data[1] = yaw_current & 0xFF;
    tx_data[2] = (pitch_current >> 8) & 0xFF;
    tx_data[3] = pitch_current & 0xFF;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}

void Gimbal_Init(void)
{

    // Yaw轴位置环：kp=8, ki=0, kd=0.1, 输出限幅±500rpm, 积分限幅0
    PID_Init(&pid_yaw_pos, 8.0f, 0.0f, 0.1f, 500.0f, -500.0f, 0.0f, 0.0f);
    // Yaw轴速度环：kp=20, ki=0.5, kd=0, 输出限幅±16384（最大电流）, 积分限幅±1000
    PID_Init(&pid_yaw_speed, 20.0f, 0.5f, 0.0f, 16384.0f, -16384.0f, 1000.0f, -1000.0f);

    // Pitch轴位置环：kp=10, ki=0, kd=0.2, 输出限幅±300rpm, 积分限幅0
    PID_Init(&pid_pitch_pos, 10.0f, 0.0f, 0.2f, 300.0f, -300.0f, 0.0f, 0.0f);
    // Pitch轴速度环：kp=25, ki=0.8, kd=0, 输出限幅±16384, 积分限幅±1000
    PID_Init(&pid_pitch_speed, 25.0f, 0.8f, 0.0f, 16384.0f, -16384.0f, 1000.0f, -1000.0f);

}



void Gimbal_PID_Control(void)
{

    // 1. 获取电机当前状态（角度转度数，转速直接用）
    float yaw_current_deg = MOTOR_ANGLE_TO_DEG(motor_feedback[YAW_MOTOR_ID].angle);
    float yaw_current_speed = motor_feedback[YAW_MOTOR_ID].speed;
    float pitch_current_deg = MOTOR_ANGLE_TO_DEG(motor_feedback[PITCH_MOTOR_ID].angle);//获取到当前电机的转动的角度，并映射到360.0f
    float pitch_current_speed = motor_feedback[PITCH_MOTOR_ID].speed;//获取到当前电机的转速（rpm)

    // 2. Yaw轴双环PID
    // 位置环：目标角度 → 速度给定
    float yaw_speed_target = PID_Calc(&pid_yaw_pos, target_yaw_deg, yaw_current_deg);
    // 速度环：速度给定 → 电流输出
    int16_t yaw_current = (int16_t)PID_Calc(&pid_yaw_speed, yaw_speed_target, yaw_current_speed);

    // 3. Pitch轴双环PID
    // 位置环：目标角度 → 速度给定
    float pitch_speed_target = PID_Calc(&pid_pitch_pos, target_pitch_deg, pitch_current_deg);
    // 速度环：速度给定 → 电流输出
    int16_t pitch_current = (int16_t)PID_Calc(&pid_pitch_speed, pitch_speed_target, pitch_current_speed);

    // 4. CAN发送电流指令
    CAN_Motor_SetVoltage(yaw_current, pitch_current);
}



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


