/**
 * @file    math_util.h
 * @brief   Small math helpers shared by the libraries (namespace math): pi, angle wrapping, 2D vector and pose.
 *          Units: mm, rad.
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

constexpr float PI = static_cast<float>(M_PI); // Float version of M_PI

constexpr float toRadian(float degrees) { return degrees * (PI / 180.0f); }

// Convert a fixed-size array of degrees to radians, including at compile time.
template <std::size_t N>
constexpr std::array<float, N> toRadians(const float (&deg)[N]) {
    std::array<float, N> out{};
    for (std::size_t i = 0; i < N; i++) out[i] = toRadian(deg[i]);
    return out;
}

struct Vec2 {
    float x, y;
};

inline float distance(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }

// Linear interpolation; t is not clamped, so values outside [0, 1] extrapolate.
constexpr Vec2 lerp(Vec2 a, Vec2 b, float t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

// Robot pose: position (mm) and heading (rad, CCW from +x).
struct Pose {
    float x, y, th;
};

// Angle wrapped into [-pi, pi]. Non-finite inputs return NaN.
float wrap(float a);

}  // namespace math
