
#include "dr16.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <tgmath.h>

#include "chassis_ctrl.h"
#include "filtering.h"
#include "stm32f4xx_hal_uart.h"


/////////////*****
///
///
///
RC_Remote_t rc_remote={0};

//DMA双缓冲区（全局，供中断使用）
uint8_t dr16_rx_buf[2][RC_FRAME_LEN]={0};


float forward_rpm,left_rpm,rotate_rpm;
float speed_rate =1.0f;
float target_speed,target_dir;  //底盘解算用；目标速度，目标方向
float target_omega;             //底盘解算用，目标自转角速度


////************************ 中断函数-----把得到的数据存放到rc_remoto ************************/
// 入参：pData→DMA接收的18字节原始数据缓冲区
void DR16_Parse(uint8_t *pData)
{
    if(pData == NULL) return;  // 空指针保护，防止程序崩溃

    memset(&rc_remote, 0, sizeof(RC_Remote_t));  // 清空数据，防止脏数据干扰

    /************************ 解析11bit摇杆通道（手册核心位域偏移） ************************/
    // CH0：偏移0bit，长度11bit → pData[0]全8bit + pData[1]低3bit | &0x07FF(11bit掩码，过滤高位)
    rc_remote.ch0 = ((uint16_t)pData[0] | ((uint16_t)pData[1] << 8)) & 0x07FF;
    // CH1：偏移11bit，长度11bit → pData[1]高5bit + pData[2]全8bit
    rc_remote.ch1 = (((uint16_t)pData[1] >> 3) | ((uint16_t)pData[2] << 5)) & 0x07FF;
    // CH2：偏移22bit，长度11bit → pData[2]高2bit + pData[3]全8bit + pData[4]低1bit
    rc_remote.ch2 = (((uint16_t)pData[2] >> 6) | ((uint16_t)pData[3] << 2) | ((uint16_t)pData[4] << 10)) & 0x07FF;
    // CH3：偏移33bit，长度11bit → pData[4]高7bit + pData[5]低4bit

    rc_remote.ch3 = (((uint16_t)pData[4] >> 1) | ((uint16_t)pData[5] << 7)) & 0x07FF;

    /************************ 解析2bit开关S1/S2（手册偏移44/46bit） ************************/
    // S1/S2均在pData[5]的高4bit，手册定义：1=上、2=下、3=中
    rc_remote.s2 = ((pData[5] >> 4) & 0x0C) >> 2;  // 取pData[5] bit6-bit7，保留高两位
    rc_remote.s1 = ((pData[5] >> 4) & 0x03);       // 取pData[5] bit4-bit5

    rc_remote.s1 =((rc_remote.s1==0)?RC_SW_MID :rc_remote.s1);
    rc_remote.s2 =((rc_remote.s2==0)?RC_SW_MID :rc_remote.s2);
}


////**********************一下是把摇杆位置转变成目标转速（rpm)*****************/////
///
///
int16_t RC2RPM(uint16_t rc_val,float max_rpm)
{
    int32_t rc_diff = 0;
    float target_rpm = 0.0f;

    //1.计算摇杆相对中点的偏差值
    rc_diff=rc_val - RC_CH_MID;

    //2.死区判断：中点附近微笑偏差，直接返回零电流（防抖）
    if (abs(rc_diff)<=RC_DEAD_ZONE)
    {
        return C620_CURR_MID;   //源代码返回C620_CURR_MID(电流）
    }

    // 3. 线性映射：摇杆偏差 -> 目标rpm（-max_rpm~max_rpm）
    target_rpm = (float)((rc_diff * max_rpm) / RC_STROKE);

    if (target_rpm>max_rpm)  target_rpm = max_rpm;
    else if (target_rpm < -max_rpm) target_rpm = -max_rpm;

    return (int16_t)target_rpm;
}




// 新增：角度归一化（-π~π），避免角度溢出
float normalize_angle(float angle)
{
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}



/////*********************
void RC_Calc_MotorSpeed(void)
{


    //左摇杆前后：前进/后退（CH3)----------向前为正
    forward_rpm = -RC2RPM(rc_remote.ch3,MAX_MOTOR_RPM)*speed_rate;
    ///右摇杆左右 :左移/右移

    ////------------------------------向右为正
    left_rpm=RC2RPM(rc_remote.ch2,MAX_MOTOR_RPM)*speed_rate;

    //左摇杆前后：前进/后退（CH3)
     rotate_rpm = RC2RPM(rc_remote.ch0,MAX_MOTOR_RPM)/2;
    ///右摇杆左右 :左移/右移


    // ---------------------- 2. 计算底盘解算核心参数 ----------------------
    // ① 目标速度（target_speed）：合成前后+左右的合速度（勾股定理）
    target_speed = sqrt(forward_rpm * forward_rpm + left_rpm * left_rpm);

    // ② 目标方向（target_dir）：合成移动方向角（弧度，底盘自身坐标系）
    // atan2(左右分量, 前后分量) → 正前=0rad，正右=π/2rad，正左=-π/2rad
    if (target_speed > 0)  // 速度>0时才计算方向（避免除0）
    {
        target_dir = atan2(left_rpm, forward_rpm);
        target_dir = normalize_angle(target_dir);  // 归一化到-π~π
    }
    else
    {
        target_dir = 0;  // 速度为0时，方向置0
    }

    // ③ 目标自转角速度（target_omega）：直接复用自转转速
    target_omega = rotate_rpm;

    //*************************2.右摇杆控制云台YAW/PITCH************************/
    switch (rc_remote.s2)
    {
        case RC_SW_UP:    speed_rate=1.0f;   break;   //高速档（100%）
        case RC_SW_MID:   speed_rate=0.5f;   break;   //中速挡 （50%）
        case RC_SW_DOWN:  speed_rate=0.2f;   break;   //低速档  (20%)
        default:          speed_rate=0.0f;   break;
    }
}
