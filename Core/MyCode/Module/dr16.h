
#ifndef BMI088_DEMO_CAN_MOTOR_H
#define BMI088_DEMO_CAN_MOTOR_H


/************************ DR16基础参数（严格对标大疆手册） ************************/
#define RC_FRAME_LEN        18u     // 一帧数据18字节
#define RC_CH_MIN           364u    // 摇杆最小值
#define RC_CH_MID           1024u   // 摇杆中立值
#define RC_CH_MAX           1684u   // 摇杆最大值

#define RC_STROKE           (RC_CH_MAX -RC_CH_MID) //摇杆单边行程：660，固定

//u表示-----无符号整数

#define RC_SW_UP            1u      // 开关上
#define RC_SW_MID           3u      // 开关中
#define RC_SW_DOWN          2u      // 开关下

#define RC_DEAD_ZONE        20u     // 摇杆防抖阈值（避免回中抖动）
#define RC_MAX_SPEED        600   //M3508最大目标转速

///RC_MAX_SPEED映射到对应-600~600的整数上

/************************C260电调宏定义（严格对标大疆手册） ************************/
#define C620_CURR_MIN   -16384   //电调最小电流
#define C620_CURR_MAX   16384    //电调最大电流
#define C620_CURR_MID   0        //电调零电流（电机停转）
#include "stm32f4xx_hal_uart.h"


/************************ 摇杆+开关结构体 ************************/
typedef struct
{
    uint16_t ch0;  // 左摇杆前后（前进/后退）
    uint16_t ch1;  // 左摇杆左右（左移/右移）
    uint16_t ch2;  // 右摇杆前后（备用）
    uint16_t ch3;  // 右摇杆左右（原地旋转）
    uint8_t  s1;   // 三位开关S1
    uint8_t  s2;   // 三位开关S2（转速档位控制）
} RC_Remote_t;
RC_Remote_t rc_remote;


/************************ 全局变量（供main.c调用） ************************/
extern RC_Remote_t rc_remote;
extern int16_t  motor_target[4];

extern uint8_t dr16_rx_buf[2][RC_FRAME_LEN];


extern  float forward_rpm,left_rpm,rotate_rpm;
extern  float speed_rate =1.0f;
extern  float target_speed,target_dir;  //底盘解算用；目标速度，目标方向
extern  float target_omega;             //底盘解算用，目标自转角速度



void DR16_Parse(uint8_t *pData);   //dr16遥控器回调接收函数
void RC_Calc_MotorSpeed(void);     //解算得到目标速度（target_speed） ，目标方向（target_dir），自转速度的公式（target_omega）




#endif //BMI088_DEMO_CAN_MOTOR_H