#ifndef BMI088_DEMO_BMI088_H
#define BMI088_DEMO_BMI088_H

#include "main.h"
#include "stm32f4xx.h"


// bmi088_driver.h 部分内容示例
#pragma once

/* 片选引脚定义 - 根据你的实际硬件连接修改 */
#define BMI088_ACC_CS_GPIO_Port    GPIOC
#define BMI088_ACC_CS_Pin          GPIO_PIN_4
#define BMI088_GYRO_CS_GPIO_Port   GPIOB
#define BMI088_GYRO_CS_Pin         GPIO_PIN_1

/* SPI 句柄定义 - 与CubeMX配置匹配 */
extern SPI_HandleTypeDef           hspi1;
#define BMI088_SPI_HANDLE          &hspi1

/* 加速度计关键寄存器地址 */
#define BMI088_ACC_CHIP_ID         0x00   //芯片 ID 寄存器（地址 0x00）
#define BMI088_ACC_PWR_CONF        0x7C   //电源配置寄存器（0x7C），配置加速度计的电源模式（比如低功耗 / 正常模式）
#define BMI088_ACC_PWR_CTRL        0x7D   //电源控制寄存器（0x7D），控制加速度计的上电 / 下电（比如写入特定值启动加速度计）。
#define BMI088_ACC_SOFTRESET       0x7E   //软复位寄存器（0x7E），写入特定值可以让加速度计执行软复位，恢复默认配置。
#define BMI088_ACC_CONF            0x40   //配置寄存器（0x40），设置加速度计的输出数据率（ODR）、滤波模式等
#define BMI088_ACC_RANGE           0x41   //量程寄存器（0x41），设置加速度计量程（比如 ±3g、±6g、±12g、±24g）
#define BMI088_ACC_X_LSB           0x12   //X 轴加速度数据低字节寄存器（0x12），加速度计的 X 轴数据分为 LSB（低 8 位）和 MSB（高 8 位）
                                          //，从这个地址开始读取可以得到完整的 X 轴数据。


/* ... 其他寄存器地址定义 */



/* 陀螺仪关键寄存器地址 */
#define BMI088_GYRO_CHIP_ID        0x00   //：陀螺仪芯片 ID 寄存器（0x00）
#define BMI088_GYRO_SOFTRESET      0x14   //陀螺仪软复位寄存器（0x14），执行陀螺仪软复位
#define BMI088_GYRO_LPM1           0x11   //低功耗模式寄存器（0x11），配置陀螺仪的低功耗模式
#define BMI088_GYRO_RANGE          0x0F   //陀螺仪量程寄存器（0x0F），设置陀螺仪的角速度量程（比如 ±2000°/s、±1000°/s 等）。
#define BMI088_GYRO_BANDWIDTH      0x10   //带宽寄存器（0x10），设置陀螺仪的滤波带宽，平衡响应速度和噪声。
#define BMI088_GYRO_RATE_X_LSB     0x02   //X 轴陀螺仪数据低字节寄存器（0x02），读取 X 轴角速度数据的起始地址。
/* ... 其他寄存器地址定义 */

/* 通信协议定义：BMI088 SPI读写指令格式 */
#define BMI088_SPI_READ_CMD        0x80   //：SPI 读指令的标志位（二进制1000 0000）。
#define BMI088_SPI_WRITE_CMD       0x00   //SPI 写指令的标志位（二进制0000 0000）

/* 函数声明 */
uint8_t BMI088_Init(void);
void BMI088_ReadAcceleration(float *ax, float *ay, float *az);
void BMI088_ReadGyroscope(float *gx, float *gy, float *gz);
static void BMI088_Acc_WriteReg(uint8_t reg, uint8_t data);
static void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data);
static void BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data, uint16_t len);
static void BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data, uint16_t len);

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} BMI088_Data_t;

extern osMessageQueueId_t BMI088DataQueueHandle;


#endif //BMI088_DEMO_BMI088_H