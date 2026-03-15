#include "filtering.h"

#include <math.h>

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


