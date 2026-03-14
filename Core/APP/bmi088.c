#include "bmi088.h"
#include <tgmath.h>
#include "cmsis_os2.h"
#include "freertos_os2.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "usb_vision.h"



#define PI 3.14159265358979323846f
// 补充：弧度/角度转换常量（1° = π/180 rad）
#define DEG_TO_RAD 0.017453292519943295f

// 首先是一些宏定义，用于数据转换
#define G 9.80665f          // 重力加速度标准值

#define Kp 1.50f            // 互补滤波比例系数
#define Ki 0.005f           // 互补滤波积分系数
#define halfT 0.0025f       // 半采样周期，单位秒。假设采样频率为200Hz，周期0.005s，halfT=0.0025

// 全局四元数，初始化为无旋转状态（代表机体坐标系与地理坐标系对齐）
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// 误差积分项，用于消除稳态误差
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
// 弧度转角度
float Yaw, Pitch, Roll;


/* 私有函数：向加速度计写入单个寄存器 */
static void BMI088_Acc_WriteReg(uint8_t reg, uint8_t data)
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
static void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
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
static void BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
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
static void BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
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



// 快速计算1.0 / sqrt(x)，著名的“平方根倒数速算法”，比库函数快很多
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;          // 邪恶的位操作：将浮点数按位解释为整数
    i = 0x5f3759df - (i >> 1);    // 魔法数字初始猜测
    y = *(float*)&i;              // 再按位解释回浮点数
    y = y * (1.5f - (halfx * y * y)); // 一次牛顿迭代提升精度
    return y;
}

// 核心姿态更新函数
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)

   ///加速度的单位是g,角速度的单位是rad/s
{
    float norm;
    float vx, vy, vz; // 理论重力向量（机体坐标系下）
    float ex, ey, ez; // 向量叉积误差

    // 1. 归一化加速度计测量值（得到单位重力向量）
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 2. 根据当前四元数，计算地理坐标系重力向量在机体坐标系下的理论投影 (vx, vy, vz)
    // 这个计算来源于四元数旋转公式，是固定的推导结果
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 3. 计算理论重力向量(v)与实际重力向量(a)的叉积，得到误差向量(e)
    // 叉积的模长|a x b| = |a||b|sinθ，当a、b均为单位向量且θ很小时，|e| ≈ θ
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 4. 对误差进行PI补偿，并修正陀螺仪读数
    // 积分误差累加，用于消除常值零偏
    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    // 将补偿量加到陀螺仪数据上，完成修正
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    // 5. 使用修正后的角速度，通过一阶龙格库塔法更新四元数
    // 这是四元数微分方程的离散化求解
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 6. 归一化四元数，防止数值计算导致其不再是单位四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 7. （可选）将四元数转换为更直观的欧拉角（单位：度）
    // 注意：此转换在俯仰角接近±90度时存在奇点，仅用于观察，控制逻辑建议直接使用四元数
    Roll = atan2f(2.0f * (q2 * q3 + q0 * q1), q0q0 - q1q1 - q2q2 + q3q3) * 57.29578f;
    Pitch = -asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.29578f;
    Yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.29578f;
}

 

void Start_BMI088_Read_Task_Task(void *argument)
{

    uint8_t init_err = BMI088_Init();

    if (init_err != 0)
    {

        osThreadTerminate(osThreadGetId());   //osThreadGetId() 获取当前运行任务的 ID，
                                              //osThreadTerminate 终止这个任务（相当于 “杀死” 当前任务）；

        return;

    }

    BMI088_Data_t angle_data;

    float ax,ay,az,gx,gy,gz;


    // 作用：给BMI088发送初始化命令（解锁、复位、配置量程/采样率），让传感器进入工作状态
    // &hspi1：告诉初始化函数“用SPI1和传感器通信”


    for (;;)
    {

        BMI088_Data_t *p_data=(BMI088_Data_t*)pvPortMalloc(sizeof(BMI088_Data_t));
        // 申请内存，用于存储传感器数据
        //p_data:指向新申请的内存地址

        if (p_data==NULL)
        {
            osDelay(10);
            continue;
        }


        BMI088_ReadAcceleration(&ax,&ay,&az);//分别得到各个方向的角速度（单位rad/s)
        BMI088_ReadGyroscope(&gx,&gy,&gz);  //分别得到各个方向的加速度（单位g)

        MahonyAHRSupdate(gx,gy,gz,ax,ay,az);

        // ③ 把姿态角赋值给结构体
        p_data->pitch = Pitch;
        p_data->roll  = Roll;
        p_data->yaw   = Yaw;

        USB_Vision_SendEuler(p_data->pitch, p_data->roll, p_data->yaw);

        osMessageQueuePut(BMI088DataQueueHandle, &p_data, 0, 0);

        osDelay(10);
    }
}