/**
 * @file    math_util.h
 * @brief   Small math helpers shared by the libraries (namespace math): pi, angle wrapping, 2D vector and pose.
 *          Units: cm, rad.
 *          Matrix helpers for an EKF would go here. (Clamping is std::clamp from <algorithm>: same code as a hand-written
 *          one on this core, and faster than fminf/fmaxf, which are library calls on a Cortex-M4.)
 */
#pragma once

#include <math.h>
#include <array>
#include <cstddef>

#ifndef M_PI  // strict -std=c++NN does not define it
#define M_PI 3.14159265358979323846
#endif

namespace math {

// M_PI is a double. Casting it once here keeps run-time maths in single precision: the Black Pill's FPU has no double
// support, and `x * M_PI` would call the software double routines.
constexpr float PI = static_cast<float>(M_PI);

// Convert a fixed-size array of degrees to radians, including at compile time.
template <std::size_t N>
constexpr std::array<float, N> toRadians(const float (&deg)[N]) {
    std::array<float, N> out{};
    for (std::size_t i = 0; i < N; i++) out[i] = deg[i] * PI / 180.0f;
    return out;
}

struct Vec2 {
    float x, y;
};

// Robot pose: position (cm) and heading (rad, CCW from +x).
struct Pose {
    float x, y, th;
};

// Angle wrapped into [-pi, pi].
float wrap(float a);

}  // namespace math
