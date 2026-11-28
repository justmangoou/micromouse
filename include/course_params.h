/**
 * @file    course_params.h
 * @brief   The block course: start pose, finish line and how the Navigator drives it. Shared by the firmware
 *          (src/course.cpp) and the host replay tool (tools/replay_run.cpp), so both behave the same.
 *
 * Coordinates: mm, x to the right along the start line, y forward from it; angles in rad, counter-clockwise.
 * Why the numbers have their values: docs/DECISIONS.md.
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

// findBlock: false for the first run (no random block; the run is recorded as the reference), true once a reference
// exists (the run with the block).
inline nav::NavParams navParams(bool findBlock) {
    nav::NavParams p;
    p.vFull = cfg::V_FULL_MM_S;
    p.track = cfg::WHEEL_TRACK_MM;

    // Driving
    p.speed = 1.0f;
    p.softProfile = true;      // slow down less in gaps (sqrt of the route's speed factor)
    p.lookahead = 85.0f;
    p.lookaheadGain = 0.18f;   // longer lookahead at speed: prevents corner cutting
    p.leftBoost = 1.30f;       // the left side 30 % faster everywhere (1.15 was test step 1)
    p.foundBoost = 1.15f;      // right wing, once the block is found (wheel commands still capped at vFull)
    // Run 2, until the block is found: slower from before the bend by T14, where it can only be seen close up.
    p.caution = findBlock ? 0.85f : 1.0f;
    p.cautionX = findBlock ? 950.0f : 1050.0f;
    p.backRange = findBlock ? 200.0f : 0.0f;  // a block found this close: brake and back off, then turn
    p.finishY = 250.0f;        // past the last block: drive onto the finish line at no less than ...
    p.finishSpeed = 0.45f;     // ... this fraction of vFull

    p.laneClose = 90.0f;       // finish lane: correct the drift off T16 / T17 with the side sensors ...
    p.laneStep = 4.0f;         // ... this far per control step ...
    p.laneLimit = 60.0f;       // ... and at most this much in all
    p.right90Mount = {cfg::TOF_MOUNT_MM[cfg::TOF_RIGHT_90][0], cfg::TOF_MOUNT_MM[cfg::TOF_RIGHT_90][1]};  // gap route

    // Trust: with the block confirmed and behind us, the rest of the course is the known map.
    p.trustSupport = 3;        // agreeing readings
    p.trustPassMm = 150.0f;
    p.trustedCaution = 1.0f;

    // Random-block detection. There is one block, and the teacher tends to put it on the path run 1 took, so only a
    // block standing on the planned route counts. Chosen by replaying the logged runs, each against another as the
    // reference: it keeps walls and mapped blocks (seen from a slightly different pose) from passing as the new block.
    nav::DetectParams& d = p.detect;
    d.maxBlocks = 1;
    d.minHitX = 1150.0f;       // the right wing starts at the pit's right wall (x = 1200)
    d.maxCentreX = 1800.0f;    // inside the arena ...
    d.minCentreX = 1160.0f;    // ... not deep in the forbidden middle (estimates of a real block reach 1190) ...
    d.clampCentreX = 1250.0f;  // ... and a block cannot stand further left than this
    d.minCentreY = -60.0f;     // ... and not far past the finish line (one on the line is centred near 0)
    d.pathReach = 150.0f;      // the hit is near the route in the next 700 mm ...
    d.centreReach = 60.0f;     // ... and the block would stand on it
    d.pathAhead = 700.0f;
    d.confirmSteps = 5;        // short for 0.17 s
    d.baselineMargin = 100.0f; // 100 mm shorter than the reference run saw from here
    if (!findBlock) d.maxBlocks = 0;
    return p;
}

}  // namespace course
