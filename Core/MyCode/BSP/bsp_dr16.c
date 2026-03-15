#include "bsp_dr16.h"

#include <stdlib.h>

#include  "usart.h"
#include <string.h>

RC_Remote_t rc_remote={0};

//DMA双缓冲区（全局，供中断使用）
uint8_t dr16_rx_buf[2][RC_FRAME_LEN]={0};
UART_HandleTypeDef *dr16_huart=NULL;

////////////////DR16初始化函数/////////////////
///入参：&huart5（你的uart5句柄）
///
void DR16_Init(UART_HandleTypeDef *huart)
{

    dr16_huart=huart;
    ///新增：清除空闲中断标志（防止初始化时触发的无效中断）
    __HAL_UART_CLEAR_IDLEFLAG(dr16_huart);
    //开启DMA接收：初始化缓冲区0，接收18字节
    HAL_UART_Receive_DMA(huart, dr16_rx_buf[0], RC_FRAME_LEN);
    ////开启UART空闲中断（核心：一帧接收完成后就出发中断，进行解析）
    __HAL_UART_ENABLE_IT(dr16_huart, UART_IT_IDLE);
}


/************************ 核心解析函数（仅摇杆+开关，严格对标手册位域） ************************/
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

