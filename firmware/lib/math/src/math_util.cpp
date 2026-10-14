/**
 * @file    math_util.cpp
 * @brief   Math helpers implementation. See math_util.h.
 */
#include "math_util.h"

namespace math {

float wrap(float a) {
    // Keep ordinary heading corrections cheap; reduce larger inputs in one step.
    constexpr float turn = 2.0f * PI;
    if (a > turn || a < -turn) a = fmodf(a, turn);
    if (a > PI) a -= turn;
    if (a < -PI) a += turn;
    return a;
}

}  // namespace math
