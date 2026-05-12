//
// Created by 34285 on 2026/5/12.
//

#include "drv_math.h"


uint16_t float_to_uint16(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    // 限制范围，防止溢出
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;

    // 线性映射计算
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
  * @brief 将浮点数映射为 32 位无符号整数
  * @param x_min   浮点数最小值
  * @param x_max   浮点数最大值
  * @return uint32_t 映射后的 32 位整数 (范围: 0 ~ 4294967295)
  */
uint32_t float_to_uint32(float x, float x_min, float x_max)
{
    // 1. 边界限制（Clamp），防止计算结果溢出 uint32_t 范围
    if (x <= x_min) return 0;
    if (x >= x_max) return 0xFFFFFFFF;

    // 2. 计算跨度
    double span = (double)x_max - (double)x_min;

    // 3. 线性映射
    // 使用 double 进行计算：(x - min) / span * (2^32 - 1)
    // 加上 0.5 是为了实现四舍五入，而不是简单的向下取整
    return (uint32_t)((((double)x - (double)x_min) * 4294967295.0 / span) + 0.5);
}
