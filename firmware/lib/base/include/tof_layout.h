#pragma once

#include <array>
#include <cmath>

namespace tof {

// Fixed storage bounds, not a robot's actual sensor or ray count.
constexpr int MAX_SENSORS = 8;
constexpr int MAX_FOV_RAYS = 9;

// Body coordinates: x forward, y left (mm), angle CCW from forward (radians).
struct Mount {
    float x = 0, y = 0, angle = 0;
};

// Caller-owned geometry and usable range. No board wiring or default mounting.
struct Layout {
    int count = 0;
    std::array<Mount, MAX_SENSORS> sensors{};
    int rayCount = 0;
    std::array<float, MAX_FOV_RAYS> rays{};  // offsets about each sensor axis, radians
    float minMm = 0, maxMm = 0;

    bool valid() const {
        if (count < 1 || count > MAX_SENSORS || rayCount < 1 || rayCount > MAX_FOV_RAYS ||
            !std::isfinite(minMm) || !std::isfinite(maxMm) || minMm < 0 || maxMm <= minMm) return false;
        for (int i = 0; i < count; ++i)
            if (!std::isfinite(sensors[i].x) || !std::isfinite(sensors[i].y) || !std::isfinite(sensors[i].angle)) return false;
        for (int i = 0; i < rayCount; ++i)
            if (!std::isfinite(rays[i])) return false;
        return true;
    }
};

}  // namespace tof
