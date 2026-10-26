#pragma once
#include "../include/config.h"
#include "micromouse.h"

inline nav::MicromouseParams robotMazeParams() {
    nav::MicromouseParams p;
    p.leftSensor = cfg::TOF_LEFT_MIDDLE;
    p.rightSensor = cfg::TOF_RIGHT_MIDDLE;
    return p;
}
