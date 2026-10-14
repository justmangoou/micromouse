/**
 * @file    math_util.cpp
 * @brief   Math helpers implementation. See math_util.h.
 */
#include "math_util.h"

namespace math {

float wrap(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

}  // namespace math
