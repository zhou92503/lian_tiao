#include "bmi088.h"

#include "cmsis_os2.h"
#include "filtering.h"
#include "freertos_os2.h"
#include "main.h"



// 私有函数：向加速度计写入单个寄存器 */
void BMI088_Acc_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    tx_buf[0] = (reg & BMI088_SPI_WRITE_CMD); // 组合写命令
    tx_buf[1] = data;

    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET); // 拉低CS，选中加速度计
    HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx_buf, 2, 10);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET); // 拉高CS，结束传输
    osDelay(1); // 短暂延时，确保时序稳定
}


/* 补充：陀螺仪写寄存器函数（之前缺失的核心实现） */
void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    // 组合写命令：寄存器地址 & 写命令位（BMI088_SPI_WRITE_CMD 是你定义的写掩码）
    tx_buf[0] = (reg & BMI088_SPI_WRITE_CMD);
    tx_buf[1] = data;

    // 拉低陀螺仪CS引脚，选中设备
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
    // 通过SPI发送写命令和数据
    HAL_SPI_Transmit(BMI088_SPI_HANDLE, tx_buf, 2, 1); // 超时时间10ms
    // 拉高CS引脚，结束传输
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    osDelay(1); // 短暂延时保证时序稳定
}

/* 私有函数：从加速度计读取多个寄存器（突发读取模式） */
void BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_cmd = reg | BMI088_SPI_READ_CMD; // 组合读命令

    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(BMI088_SPI_HANDLE, &tx_cmd, 1, 1);
    // 注意：根据数据手册，加速度计读操作时，第一个接收字节是“哑元”（dummy），需要丢弃
    uint8_t dummy;

    HAL_SPI_Receive(BMI088_SPI_HANDLE, &dummy, 1, 1);
    // 然后连续读取len个字节的数据

    HAL_SPI_Receive(BMI088_SPI_HANDLE, data, len, 5);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
}


/* 陀螺仪的读写函数实现类似，但注意其读操作不需要丢弃第一个哑元字节 */
void BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{

    uint8_t tx_cmd = reg | BMI088_SPI_READ_CMD;

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(BMI088_SPI_HANDLE, &tx_cmd, 1, 1);

    // 陀螺仪直接读取有效数据

    HAL_SPI_Receive(BMI088_SPI_HANDLE, data, len, 5);

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}



uint8_t BMI088_Init(void)
{
    uint8_t error_flag = 0;

    uint8_t chip_id = 0;       //定义临时变量，用于存储读取到的加速度计 / 陀螺仪芯片 ID。

    /* 1. 验证加速度计芯片ID */
    BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID, &chip_id, 1);

    if(chip_id != 0x1E)
    { // BMI088加速度计芯片ID固定为0x1E
        error_flag |= 0x01; // 标记加速度计ID错误
        // 在实际项目中，这里可以加入日志输出或LED指示
    }


    /* 2. 验证陀螺仪芯片ID */
    BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if(chip_id != 0x0F)
    { // BMI088陀螺仪芯片ID固定为0x0F
        error_flag |= 0x02; // 标记陀螺仪ID错误
    }


    /* 3. 加速度计初始化序列 */
    // 软复位
    BMI088_Acc_WriteReg(BMI088_ACC_SOFTRESET, 0xB6);
    osDelay(50); // 等待复位完成
    // 上电

    BMI088_Acc_WriteReg(BMI088_ACC_PWR_CTRL, 0x04);
    osDelay(50); // 等待电源稳定
    // 切换到活跃模式

    BMI088_Acc_WriteReg(BMI088_ACC_PWR_CONF, 0x00);
    osDelay(10);

    /* 4. 陀螺仪初始化序列 */
    // 软复位
    BMI088_Gyro_WriteReg(BMI088_GYRO_SOFTRESET, 0xB6);
    osDelay(50);

    // 切换到正常模式（退出深度睡眠）
    BMI088_Gyro_WriteReg(BMI088_GYRO_LPM1, 0x00);
    osDelay(10);

    /* 5. 配置传感器参数 */
    // 配置加速度计：±6g量程，输出数据率800Hz，正常模式滤波器
    BMI088_Acc_WriteReg(BMI088_ACC_RANGE, 0x01); // 0x01对应±6g
    BMI088_Acc_WriteReg(BMI088_ACC_CONF, 0xAC); // 0xAC对应ODR=800Hz, BWP=Normal

    // 配置陀螺仪：±500°/s量程，2000Hz ODR, 532Hz带宽
    BMI088_Gyro_WriteReg(BMI088_GYRO_RANGE, 0x02); // 0x02对应±500°/s
    BMI088_Gyro_WriteReg(BMI088_GYRO_BANDWIDTH, 0x00); // 0x00对应ODR=2000Hz, BW=532Hz

    osDelay(100); // 等待配置生效
    return error_flag; // 返回错误标志，0表示初始化成功
}



void BMI088_ReadAcceleration(float *ax, float *ay, float *az)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;
    float sensitivity; // 灵敏度，单位: LSB/g

    // 读取6个字节的加速度数据 (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    BMI088_Acc_ReadReg(BMI088_ACC_X_LSB, buffer, 6);//把数据读取存放到buffer的字节组（6个字节)


    // 组合成16位有符号整数
    raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);


    // 根据当前设置的量程选择灵敏度。例如，±6g量程时，灵敏度为 32768 / 6 ≈ 5461.3 LSB/g
    // 实际项目中，这里可以读取ACC_RANGE寄存器动态确定灵敏度
    sensitivity = 5461.3f;

    // 转换为重力加速度g
    *ax = raw_x / sensitivity;
    *ay = raw_y / sensitivity;
    *az = raw_z / sensitivity;
}


void BMI088_ReadGyroscope(float *gx, float *gy, float *gz)
{
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;
    float sensitivity; // 灵敏度，单位: LSB/(°/s)

    // 读取6个字节的陀螺仪数据
    BMI088_Gyro_ReadReg(BMI088_GYRO_RATE_X_LSB, buffer, 6);

    raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    // 根据当前设置的量程选择灵敏度。例如，±500°/s量程时，灵敏度为 32768 / 500 ≈ 65.536 LSB/(°/s)
    sensitivity = 65.536f;

    // 转换为度每秒 (°/s)
    float gx_deg = raw_x / sensitivity;
    float gy_deg = raw_y / sensitivity;
    float gz_deg= raw_z / sensitivity;

    //这里得到的角速度是弧度/秒，需要转换成角度/秒

    *gx=gx_deg*DEG_TO_RAD;
    *gy=gy_deg*DEG_TO_RAD;
    *gz=gz_deg*DEG_TO_RAD;
}



