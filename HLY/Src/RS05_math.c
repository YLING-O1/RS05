//
// Created by 34285 on 2026/5/7.
//

#include "RS05_math.h"

//float数据类型转换为uint类型
uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;

    return (uint32_t)((double)(x - offset) * (double)((1ULL << bits) - 1) / (double)span);
}
