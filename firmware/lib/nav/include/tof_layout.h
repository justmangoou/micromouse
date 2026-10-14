/**
 * @file    tof_layout.h
 * @brief   The one place the ToF sensor layout is defined. Everything else (navigator, tof_array, pins, host tests) reads
 *          it from here; change the layout here only.
 *
 * @par Conventions
 *   - Sensor k is the one wired to XSHUT pin k (pins.h): 0..3 = right edge, right middle, left middle, left edge.
 *   - Body frame: x forward, y left, mm. Angles are measured from the heading, counter-clockwise positive, so the right
 *     side is negative.
 *   - The sim's default layout is in sim/config.py (TOF_ANGLES, TOF_MOUNT, TOF_FOV, TOF_MIN, TOF_MAX). The sim can run
 *     other layouts for experiments, so it keeps its own copy; `python -m sim.tools.check_firmware` fails if the two
 *     defaults drift apart.
 */
#pragma once

#include <array>
#include "math_util.h"

namespace tof {

constexpr int COUNT = 4;
constexpr int RIGHT_EDGE = 0, RIGHT_MIDDLE = 1, LEFT_MIDDLE = 2, LEFT_EDGE = 3;  // sensor indices

constexpr float ANGLE_DEG[COUNT] = {-60.0f, -20.0f, 20.0f, 60.0f};
constexpr float MOUNT_MM[COUNT][2] = {{60.0f, -45.0f}, {60.0f, -20.0f}, {60.0f, 20.0f}, {60.0f, 45.0f}};

// Each of the 4 sensors sees a cone about 25 deg wide; it is modelled as these rays around the sensor axis, nearest hit
// wins. (Five rays per sensor, not five sensors.)
constexpr int FOV_RAYS = 5;
constexpr float FOV_DEG[FOV_RAYS] = {-12.0f, -6.0f, 0.0f, 6.0f, 12.0f};

constexpr float MIN_MM = 30.0f;    // closest distance the sensor reports
constexpr float MAX_MM = 1200.0f;  // "nothing in range"

constexpr std::array<float, COUNT> ANGLE = math::toRadians(ANGLE_DEG);   // rad
constexpr std::array<float, FOV_RAYS> FOV = math::toRadians(FOV_DEG);    // rad

}  // namespace tof
