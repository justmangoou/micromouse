/**
 * @file    course_params.h
 * @brief   The block course: start pose, finish line and how the Navigator drives it. Shared by the firmware
 *          (src/course.cpp) and the host replay tool (tools/replay_run.cpp), so both behave the same.
 *
 * Coordinates: mm, x to the right along the start line, y forward from it; angles in rad, counter-clockwise.
 * Why each number has its value: docs/DECISIONS.md.
 */
#pragma once

#include "config.h"
#include "navigator.h"

namespace course {

// Start: behind the start line in the left column, facing forward (+y).
constexpr float START_X = 300.0f;
constexpr float START_Y = -80.0f;
constexpr float START_TH = 1.57079633f;

// Finish: the car is past the finish line (y < -90) anywhere in the right column. Crossing it anywhere wins.
constexpr float FINISH_Y = -90.0f;
constexpr float FINISH_X_MIN = 1215.0f, FINISH_X_MAX = 1750.0f;
constexpr float FINISH_MIN_PROGRESS = 0.8f;  // and most of the route is done (not a shortcut)

inline nav::NavParams navParams() {
    nav::NavParams p;
    p.vFull = cfg::V_FULL_MM_S;
    p.track = cfg::WHEEL_TRACK_MM;

    // Driving
    p.speed = 0.95f;           // ~590 mm/s on clear sections; fast in the mapped left/top zones
    p.softProfile = true;      // slow down less in gaps (sqrt of the route's speed factor)
    p.lookaheadGain = 0.24f;   // longer lookahead at speed: prevents corner cutting at high speed
    p.caution = 0.68f;         // slow down only when entering right wing (x > 1050 mm) for unmapped block
    p.finishY = 250.0f;        // past the last block: drive onto the finish line at no less than ...
    p.finishSpeed = 0.45f;     // ... this fraction of vFull

    // Random-block detection: the course has one block that is not on the map, in the right wing, put in our way.
    nav::DetectParams& d = p.detect;
    d.maxBlocks = 1;
    d.minHitX = 1150.0f;       // the right wing starts at the pit's right wall (x = 1200)
    d.maxCentreX = 1800.0f;    // inside the arena ...
    d.minCentreY = 60.0f;      // ... and not on the finish tape
    d.pathReach = 150.0f;      // only blocks on the route in the next 700 mm matter
    d.pathAhead = 700.0f;
    d.confirmSteps = 3;        // short for 0.1 s
    d.baselineMargin = 100.0f; // 100 mm shorter than every clean run saw from here
    return p;
}

}  // namespace course
