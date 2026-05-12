//
// Created by 34285 on 2026/5/12.
//

#ifndef RS05_DRV_MATH_H
#define RS05_DRV_MATH_H

#include <stdint.h>

uint16_t float_to_uint16(float x, float x_min, float x_max, int bits);

uint32_t float_to_uint32(float x, float x_min, float x_max);

#endif //RS05_DRV_MATH_H
