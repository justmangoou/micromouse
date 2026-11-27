/**
 * @file    run_record.h
 * @brief   Records each course run in flash so it can be read after an untethered run (tools/read_run.py).
 *
 * Two slots of 6 KB at the top of the 64 KB flash:
 *   - slot 0, run 1: the course without the random block. Once such a run reaches the finish with no fault, its log is
 *     the reference for block detection until power-off, a reset, or holding KEY. The log stays in flash, but after a
 *     restart it is not used: the first run learns again.
 *   - slot 1, run 2: every run made while a reference exists.
 * A slot holds
 *   - a summary and a trace (pose every 400 ms, state, faults, blocks found), written once when the run ends, and
 *   - a fast log, one 16-byte record per control step (30 Hz, up to 11.7 s; later steps are not logged): pose, steering
 *     angle, wheel targets and measured wheel speeds, the three ToF ranges.
 * The slot is erased during the 3 s start countdown, one page per control step, while the motors are off: a page erase
 * stalls the CPU for ~30 ms.
 *
 * It also remembers, in a backup register that survives a reset, whether a run was in progress: if the chip resets in
 * the middle of a run, the next boot reports it.
 */
#pragma once

#include <stdint.h>

#include "math_util.h"
#include "navigator.h"
#include "reliability.h"

namespace runrec {

// Once at boot. True when the chip reset in the middle of the last run.
bool boot();

// The reference run (see above), for Navigator::setBaseline. Empty when none was recorded since power-on.
nav::Baseline reference();
void clearReference();

// During the start countdown: pick the slot (prepare), then erase it one page per control step (eraseStep).
void prepare();
void eraseStep();

// At the start of the run: forget the previous run's trace.
void startRun();

// Every control step.
void markRunning(bool running);
void logStep(uint32_t msSinceStart, const math::Pose& pose, float alpha, const float targetMmS[2], const float tofMm[]);
void traceStep(const reliability::Supervisor& supervisor, const nav::Navigator& navigator, const math::Pose& pose,
               uint32_t now);

// Writes the summary and trace once the run has ended (Done or Fault). True when it wrote (the write blocks briefly).
// reachedFinish: the run crossed the finish line (not the time limit); such a run 1 becomes the reference.
bool saveIfEnded(const reliability::Supervisor& supervisor, bool reachedFinish);

}  // namespace runrec
