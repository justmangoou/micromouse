/**
 * @file    course.cpp
 * @brief   Block-course mode (env:course_nav): plan with A* on the stored map, follow the path, replan around
 *          blocks the map does not have, stop over the finish line.
 */
#include <cmath>
#include <string.h>

#include "baseline.h"
#include "board.h"
#include "config.h"
#include "map_data.h"
#include "motor.h"
#include "navigator.h"
#include "robot.h"

namespace {

// Start pose: behind the start line, facing +y (mm, rad)
constexpr float START_X = 300.0f;
constexpr float START_Y = -80.0f;
constexpr float START_TH = 1.57079633f;

// Finish: the whole car below the line in the right-hand column, most of the path done
constexpr float FINISH_Y = -90.0f;
constexpr float FINISH_X_MIN = 1215.0f, FINISH_X_MAX = 1750.0f;  // anywhere on the line wins, no need to be central
constexpr float FINISH_MIN_PROGRESS = 0.8f;

nav::NavParams navParams() {
    nav::NavParams p;
    p.vFull = cfg::V_FULL_MM_S;
    p.track = cfg::WHEEL_TRACK_MM;
    p.speed = 0.88f;  // fraction of vFull on a clear straight (~550 mm/s). 0.70 = 435 until the speed loop was fixed on 2026-11-21; 0.85 ran clean in 8.9 s. The host closed-loop tests overlap a block at 0.90 and above (0.88 passes), so the next gain needs better tracking, not a bigger number. The path's clearance/curvature factors scale it down
                      // (about x0.6 in the tight gaps), and the floor is 0.22 vFull. 0.35 and 0.45 both ran at that floor, ~145 mm/s.
    p.lookaheadGain = 0.22f;  // 0.15 wobbled on the long straight at ~450 mm/s, 0.30 cut the corner at T9: in between
    p.finishY = 250.0f;     // past the last block (T16 at y 150): the run in to the line at 0.45 vFull instead of the ~137 mm/s floor
    p.finishSpeed = 0.45f;
    p.softProfile = true;  // sqrt of the clearance/curvature factor: ~0.63 instead of 0.4 in the tight gaps
    p.caution = 1.0f; // no unmapped blocks to be careful about on this course (was 0.6 for x > 1050)
    // Random-block detection (2026-11-21): ONE random block exists, in the right wing. scan() now knows the arena and pit
    // walls, only accepts a block whose centre is inside the arena and above the finish tape, and keeps at most one.
    // Check a run with NO extra block first: gCourseTrace.unmapped must stay 0.
    p.unmappedMinX = 1150.0f;   // the right wing starts at the pit's right wall, x = 1200
    p.unmappedMaxX = 1800.0f;
    p.unmappedMinY = 60.0f;
    p.maxExtras = 1;
    p.pathReach = 150.0f;   // only blocks on our way: the planned path in the next 700 mm
    p.pathAhead = 700.0f;
    p.confirmSteps = 3;     // 0.1 s of consistent short readings
    // Phantoms seen on clean runs (a run with no random block found one at (1341, 589) and detoured): the pit's top-right
    // corner and nearby walls read short when the pose is a few cm off. scan() now ignores hits within 150 mm of a wall, and
    // include/background.h marks where clean runs returned unexplained hits (tools/make_background.py). A random block in the
    // corridor beside the pit (within 150 mm of T15/T16 or the walls) is NOT detectable: those hits are ignored on purpose.
    // History: with the first version of this (any short range at x > 1000 is a block) a run with no extra block found
    // 8 phantoms and 36 replans: the arena wall (blocks at x 1915), the pit's right wall (blocks at (1138,193) and
    // (1139,357)) and the finish tape. The run took 21.6 s instead of 9.9 s. If phantoms come back, set
    // p.unmappedMinX = 1.0e9f to switch detection off.
    return p;
}

nav::Navigator gNav(cfg::TOF_LAYOUT, navParams());  // ~36 KB of planner arrays, static on purpose

// Where the car went, for ST-Link after an untethered run: one sample every 400 ms from the start (19 s). x, y in mm,
// heading in mrad. state and fault are reliability::State / Fault.
extern "C" uint32_t _end;  // end of .bss: the stack grows down toward it
constexpr uint32_t STACK_PAINT = 0xA5A5A5A5u;

constexpr int TRACE_N = 48;
struct CourseTrace {
    uint32_t samples;
    uint32_t replans;
    uint32_t unmapped;    // unmapped blocks found
    uint32_t doneAtMs;    // ms after the start when the run ended (0 = still running)
    uint32_t stackFree;   // bytes between the end of .bss and the deepest the stack has reached (the heap shares it)
    uint8_t state, fault; // latest
    uint8_t lastFault;    // the last fault of the run (0 = none; 8 = the chip reset during the run)
    uint8_t resetFlags;   // RCC_CSR bits 31..24 at boot: 27 power-on / brown-out, 26 reset pin, 28 software, 29/30 watchdog
    uint8_t countedExtras;
    int16_t extraX[nav::MAX_EXTRA], extraY[nav::MAX_EXTRA];  // where the unmapped blocks were found, mm
    int16_t x[TRACE_N], y[TRACE_N], thMrad[TRACE_N];
    uint8_t st[TRACE_N];  // state << 4 | fault
};
volatile CourseTrace gCourseTrace = {};

uint32_t* gPaintTop = nullptr;  // top of the painted area, just below where the stack was when runMode started

// Run records survive a reset or a power-off in the top 10 KB of the 64 KB flash: two slots of 5 pages, and each run goes into
// the older slot, so the last TWO runs are always kept (the 3rd run overwrites the 1st). A slot holds, from its start:
//   0..511     header (magic, run number) + the 400 ms trace (CourseTrace), written when the run ends
//   512..5119  the fast log: 16-byte records at 30 Hz, 288 of them = 9.5 s, written while Running:
//              ms since the start (u16), x, y, heading mrad (i16 each), steering angle alpha (i8, x10 mrad), wheel targets L R and
//              measured L R (i8 each, x5 mm/s), ToF right-45, front, left-45 (u8 each, x5 mm). Erased flash reads 0xFF.
// Read both with tools/read_run.py. The slot is erased during the start countdown (motors off, one page per control step: a page
// erase stalls the CPU for ~30 ms) and written a few halfwords per step (~50 us each).
extern "C" uint32_t _sidata, _sdata, _edata;
constexpr uint32_t SLOT_ADDR[2] = {0x0800D800, 0x0800EC00};
constexpr uint32_t SLOT_PAGES = 5;
constexpr uint32_t SLOT_TRACE_BYTES = 512;
constexpr uint32_t REC_BYTES = 16;
constexpr uint32_t LOG_MAX = (SLOT_PAGES * 1024 - SLOT_TRACE_BYTES) / REC_BYTES;  // 288
constexpr uint32_t SLOT_MAGIC = 0xC0DE0002;
struct SlotHeader { uint32_t magic, seq; };
static_assert(sizeof(SlotHeader) + sizeof(CourseTrace) <= SLOT_TRACE_BYTES, "the run record must fit its half page");
static_assert(cfg::TOF_COUNT == 3, "the run log record has three ToF fields");
uint32_t gSlot = 0, gSeq = 0;
uint32_t gLogN = 0;
bool gLogOk = false;     // the slot is erased and the log may be written
bool gLogArmed = false;  // the program does not reach into the slots
uint32_t gLogErased = 0;
bool gTraceSaved = true;   // the record of the current run is in flash (true until a run starts)

uint32_t slotSeq(uint32_t slot) {
    const SlotHeader* h = reinterpret_cast<const SlotHeader*>(SLOT_ADDR[slot]);
    return h->magic == SLOT_MAGIC ? h->seq : 0;
}

// Pick the older slot for the coming run. Erasing then goes one page per control step during the countdown.
void logBegin() {
    uint32_t sa = slotSeq(0), sb = slotSeq(1);
    gSlot = sa <= sb ? 0 : 1;
    gSeq = (sa > sb ? sa : sb) + 1;
    gLogN = 0;
    gLogErased = 0;
    gLogOk = false;
    gTraceSaved = false;
    // The code and the initialised data end at _sidata + their size: refuse to touch the slots if the program grew into them.
    uint32_t codeEnd = reinterpret_cast<uint32_t>(&_sidata) + static_cast<uint32_t>((&_edata - &_sdata) * 4);
    gLogArmed = codeEnd <= SLOT_ADDR[0];
}

void logEraseStep() {
    if (!gLogArmed || gLogErased >= SLOT_PAGES) return;
    FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_PAGES, FLASH_BANK_1, SLOT_ADDR[gSlot] + gLogErased * 1024, 1};
    uint32_t pageError = 0;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef ok = HAL_FLASHEx_Erase(&erase, &pageError);
    HAL_FLASH_Lock();
    if (ok != HAL_OK) {
        gLogArmed = false;
        return;
    }
    if (++gLogErased == SLOT_PAGES) gLogOk = true;
}

int8_t q8(float v, float unit) {
    float q = v / unit;
    return static_cast<int8_t>(q > 127.0f ? 127 : q < -128.0f ? -128 : static_cast<int>(q < 0 ? q - 0.5f : q + 0.5f));
}
uint8_t u8(float mm) {
    float q = mm / 5.0f;
    return static_cast<uint8_t>(q > 255.0f ? 255 : q < 0.0f ? 0 : static_cast<int>(q + 0.5f));
}
int16_t sat16(float v) { return static_cast<int16_t>(v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : v); }

void flashProgram(uint32_t addr, const uint8_t* data, uint32_t bytes) {  // halfwords, little endian; the area must be erased
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < bytes; i += 2) {
        uint16_t h = static_cast<uint16_t>(data[i] | ((i + 1 < bytes ? data[i + 1] : 0xFF) << 8));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, h);
    }
    HAL_FLASH_Lock();
}

void logAppend(uint32_t ms, const math::Pose& pose, float alpha, const float target[2], const float tof[]) {
    if (!gLogOk || gLogN >= LOG_MAX) return;
    float ml, mr;
    motorSpeeds(&ml, &mr);
    int16_t x = sat16(pose.x), y = sat16(pose.y), th = sat16(pose.th * 1000.0f);
    uint16_t t = static_cast<uint16_t>(ms & 0x7FFF);  // bit 15 clear: a written record never starts with 0xFFFF
    uint8_t rec[REC_BYTES] = {
        static_cast<uint8_t>(t), static_cast<uint8_t>(t >> 8), static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8),
        static_cast<uint8_t>(y), static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(th), static_cast<uint8_t>(th >> 8),
        static_cast<uint8_t>(q8(alpha, 0.01f)), static_cast<uint8_t>(q8(target[0], 5.0f)), static_cast<uint8_t>(q8(target[1], 5.0f)),
        static_cast<uint8_t>(q8(ml, 5.0f)), static_cast<uint8_t>(q8(mr, 5.0f)), u8(tof[0]), u8(tof[1]), u8(tof[2]),
    };
    flashProgram(SLOT_ADDR[gSlot] + SLOT_TRACE_BYTES + gLogN * REC_BYTES, rec, REC_BYTES);
    ++gLogN;
}

// When a run ends: header + trace into the slot's first half page (erased at the start of the run).
void saveTraceToFlash() {
    if (!gLogArmed || !gLogOk) return;
    uint8_t buf[sizeof(SlotHeader) + sizeof(CourseTrace)];
    SlotHeader h = {SLOT_MAGIC, gSeq};
    memcpy(buf, &h, sizeof(h));
    memcpy(buf + sizeof(h), const_cast<const CourseTrace*>(&gCourseTrace), sizeof(CourseTrace));
    flashProgram(SLOT_ADDR[gSlot], buf, sizeof(buf));
}

void paintStack() {
    uint32_t* p = &_end;
    gPaintTop = reinterpret_cast<uint32_t*>(__get_MSP()) - 16;
    while (p < gPaintTop) *p++ = STACK_PAINT;
}

void traceStep(const reliability::Supervisor& sup, const math::Pose& pose, uint32_t now) {
    static uint32_t lastAt = 0, lastScan = 0;
    if (now - lastScan >= 1000) {  // a once-a-second scan of the unused stack
        lastScan = now;
        // From the top of the painted area down: the first painted word is as deep as the stack has gone. The heap
        // (printf) grows up from _end, so the gap below is shared between the two. Stack depth since runMode, in bytes.
        uint32_t* p = gPaintTop;
        while (p > &_end && *(p - 1) != STACK_PAINT) --p;
        gCourseTrace.stackFree = static_cast<uint32_t>(reinterpret_cast<uint8_t*>(p) - reinterpret_cast<uint8_t*>(&_end));
    }
    gCourseTrace.state = static_cast<uint8_t>(sup.state());
    gCourseTrace.fault = static_cast<uint8_t>(sup.fault());
    if (sup.started() && sup.fault() != reliability::Fault::None) gCourseTrace.lastFault = gCourseTrace.fault;
    gCourseTrace.replans = gNav.replans();
    gCourseTrace.unmapped = gNav.unmappedBlocks();
    gCourseTrace.countedExtras = static_cast<uint8_t>(gNav.unmappedBlocks());
    for (int i = 0; i < gNav.unmappedBlocks() && i < nav::MAX_EXTRA; ++i) {
        gCourseTrace.extraX[i] = static_cast<int16_t>(gNav.planner().extra(i).x);
        gCourseTrace.extraY[i] = static_cast<int16_t>(gNav.planner().extra(i).y);
    }
    if (!sup.started()) return;
    bool finished = sup.state() == reliability::State::Done || sup.state() == reliability::State::Fault;
    if (finished && gCourseTrace.doneAtMs == 0) gCourseTrace.doneAtMs = now - sup.startedAt();
    uint32_t n = gCourseTrace.samples;
    if (n >= TRACE_N || (!finished && now - lastAt < 400 && n > 0)) return;
    if (finished && gCourseTrace.doneAtMs != now - sup.startedAt() && n > 0) return;  // one last sample at the end
    lastAt = now;
    gCourseTrace.x[n] = static_cast<int16_t>(pose.x);
    gCourseTrace.y[n] = static_cast<int16_t>(pose.y);
    gCourseTrace.thMrad[n] = static_cast<int16_t>(pose.th * 1000.0f);
    gCourseTrace.st[n] = static_cast<uint8_t>((gCourseTrace.state << 4) | gCourseTrace.fault);
    gCourseTrace.samples = n + 1;
}

// "A run is in progress" survives a reset in the backup register DR1 (the backup domain is kept through NRST, software
// and watchdog resets, and through most brown-outs). If the chip resets mid-run, the next boot sees the flag still set.
constexpr uint16_t RUN_MARK = 0xA55A;
constexpr uint8_t FAULT_RESET_DURING_RUN = 8;
bool gRunMarked = false;

void markRun(bool on) {
    if (on == gRunMarked) return;
    BKP->DR1 = on ? RUN_MARK : 0;
    gRunMarked = on;
}

// Status LED: solid = running or finished, slow blink = ready, N quick flashes then a pause = fault number N (1 sensor,
// 2 timing, 3 command expired, 4 no route, 5 pose, 6 configuration, 7 timeout, 8 the chip reset during a run). The last fault of a run stays on
// until the next run starts, so it can be read off the robot without a debugger.
uint8_t gLastFault = 0;

bool statusLed(const reliability::Supervisor& sup, uint32_t now) {
    using reliability::State;
    if (sup.state() == State::Running) { gLastFault = 0; return true; }
    if (sup.started() && sup.fault() != reliability::Fault::None) gLastFault = static_cast<uint8_t>(sup.fault());
    uint8_t code = sup.fault() != reliability::Fault::None ? static_cast<uint8_t>(sup.fault()) : gLastFault;
    if (code != 0) {  // N flashes of 150 ms, 150 ms apart, then a 1.2 s pause
        uint32_t period = code * 300u + 1200u, t = now % period;
        return t < code * 300u && (t % 300u) < 150u;
    }
    if (sup.state() == State::Done) return true;
    return (now / 500) % 2 == 0;  // ready: slow blink
}

// Sensor health for the course. The floor is open (no walls), so "nothing in range" is a normal reading and clear space.
// A sensor only has to be alive: a good range or a no-target result. Missing, stale, bus-error and not-ready are faults.
reliability::Fault courseIssue(const SensorStep& s, bool initialStart) {
    reliability::Fault f = sensorIssue(s, 0, initialStart);  // gyro, timing, start-up checks; no required valid range
    if (f != reliability::Fault::None) return f;
    for (int i = 0; i < TOF_COUNT; ++i) {
        reliability::RangeStatus st = s.ranges[i].status;
        if (st != reliability::RangeStatus::Valid && st != reliability::RangeStatus::Invalid) return reliability::Fault::Sensor;
    }
    return reliability::Fault::None;
}

bool atFinish(const math::Pose& p) {
    return p.y < FINISH_Y && p.x > FINISH_X_MIN && p.x < FINISH_X_MAX && gNav.progress() > FINISH_MIN_PROGRESS;
}

}  // namespace

void runMode(const StartupStatus&) {
    using namespace reliability;
    paintStack();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    gCourseTrace.resetFlags = static_cast<uint8_t>(RCC->CSR >> 24);
    RCC->CSR |= RCC_CSR_RMVF;
    if (BKP->DR1 == RUN_MARK) {  // the last run never ended: the chip reset in the middle of it
        gLastFault = FAULT_RESET_DURING_RUN;
        gCourseTrace.lastFault = FAULT_RESET_DURING_RUN;
        BKP->DR1 = 0;
    }
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    // gNav.setBackground(...): off (the geometric rules + the baseline below replace it).
    {
        static_assert(sizeof(BaselineEntry) == sizeof(nav::Navigator::BaselineEntry), "baseline.h and the Navigator must agree on the entry layout");
        nav::Navigator::Baseline b;
        b.e = reinterpret_cast<const nav::Navigator::BaselineEntry*>(BASE_TABLE);
        b.n = BASE_N;
        b.cell = BASE_CELL;
        b.bin = BASE_BIN;
        b.margin = 100.0f;
        gNav.setBaseline(b);
    }
    nav::FusedOdometry odo(cfg::ODOM);
    odo.reset({START_X, START_Y, START_TH});
    Supervisor supervisor;
    if (!gNav.configurationValid()) supervisor.latch(Fault::Configuration);
    bool planned = false;
    bool routeReady = false;
    uint32_t lastPlan = HAL_GetTick() - RETRY_MS;
    unsigned pressed = 0;
    bool counting = false;      // button pressed: 3 s to step away before the run starts
    bool autoStart = false;     // re-armed after a finished run: start as soon as ready
    uint32_t startAt = 0;
    SensorStep s;

    while (true) {
        waitStep(s);
        float target[2] = {};  // wheel targets this step, mm/s
        State before = supervisor.state();
        if (before != State::Done && before != State::Fault && !updateOdometry(odo, s)) supervisor.latch(Fault::Pose);
        Fault issue = courseIssue(s, !supervisor.started());
        if (before == State::Running && !motorsEnabled()) issue = Fault::CommandExpired;
        if (issue != Fault::None) routeReady = false;
        if (issue == Fault::None && (!routeReady || !gNav.hasRoute())) issue = Fault::NoRoute;
        pressed = buttonPressed() ? pressed + 1 : 0;
        if (before == State::Done && pressed >= 3) {
            // The run finished and the robot has been carried back to the start line: this press re-arms it and starts the next
            // run as soon as it is ready again (stands still and has planned). A fault stays latched until reset.
            supervisor = Supervisor();
            odo.reset({START_X, START_Y, START_TH});
            routeReady = false;
            planned = false;
            counting = false;
            autoStart = true;
        }
        if (supervisor.state() != State::Ready) counting = false;
        if (!counting && supervisor.state() == State::Ready && (pressed >= 3 || autoStart)) {
            autoStart = false;
            counting = true;
            startAt = s.completedAt + 3000;  // time to take your hands away from the sensors
            logBegin();
        }
        bool startNow = counting && static_cast<int32_t>(s.completedAt - startAt) >= 0;
        if (counting && !startNow) logEraseStep();
        if (startNow) {
            // The button press is a fresh start: whatever the wheels or the gyro did while the robot was being placed
            // is dropped. Pose and heading go back to the start line, the plan is made from there and the run record
            // starts empty.
            counting = false;
            odo.reset({START_X, START_Y, START_TH});
            routeReady = gNav.start(odo.pose, 0.0f);
            planned = true;
            noteStall();  // the plan blocked the loop
            for (int i = 0; i < TRACE_N; ++i) {  // no leftovers of the previous run
                gCourseTrace.x[i] = gCourseTrace.y[i] = gCourseTrace.thMrad[i] = 0;
                gCourseTrace.st[i] = 0;
            }
            gCourseTrace.samples = 0;
            gCourseTrace.doneAtMs = 0;
            gCourseTrace.lastFault = 0;
            if (issue == Fault::None && !routeReady) issue = Fault::NoRoute;
        }
        supervisor.update(s.completedAt, issue, s.countsL == 0 && s.countsR == 0, startNow);
        if (supervisor.state() != State::Fault && supervisor.started() && (s.completedAt - supervisor.startedAt()) * 0.001f > cfg::RUN_TIME_LIMIT_S)
            supervisor.done();

        if (supervisor.state() != State::Running) motorEnable(false);
        if (supervisor.state() == State::Recovering) {
            if (recoverSensors(s)) {
                supervisor.update(HAL_GetTick(), Fault::Sensor, false, false);
            } else if (courseIssue(s, !supervisor.started()) == Fault::None && !routeReady &&
                       s.completedAt - lastPlan >= RETRY_MS) {
                lastPlan = s.completedAt;
                ++gRuntime.recoveryAttempts;
                float planTime = supervisor.started() ? (s.completedAt - supervisor.startedAt()) * 0.001f : 0;
                if (!planned) { routeReady = gNav.start(odo.pose, planTime); planned = true; }
                else routeReady = gNav.reposition(odo.pose, planTime);
                noteStall();  // the plan blocked the loop; the next step is not a timing fault
            }
        } else if (supervisor.state() == State::Running) {
            if (before != State::Running) motorEnable(true);
            float t = (s.completedAt - supervisor.startedAt()) * 0.001f;
            float duty[2] = {};
            int replansBefore = gNav.replans();
            if (gNav.replanDue(t)) motorExtendLease(600);  // the plan blocks the loop: do not let the motor command expire meanwhile
            // gNav.localize(odo.pose, s.tof, t);  // off: on the real course the blocks are a few cm off the map, and it
            //                                      // pulled the pose by ~5 cm in one step (see run record). Re-enable once the map is measured.
            gNav.step(odo.pose, odo.v, s.tof, t, duty);
            // A replan runs the planner inside this step. Without an FPU that blocks the loop for ~150 ms: the motors'
            // command lease ran out (they coast to a stop) and this snapshot is older than the usual limit. That is
            // expected here and not a fault, so allow it once and restart the motors.
            if (gNav.replans() != replansBefore) {
                s.maxAgeMs = cfg::STALL_MAX_AGE_MS;
                noteStall();
                if (!motorsEnabled()) motorEnable(true);
            }
            Fault after = courseIssue(s, !supervisor.started());
            if (!gNav.hasRoute()) after = Fault::NoRoute;
            if (!motorsEnabled()) after = Fault::CommandExpired;
            if (!std::isfinite(duty[0]) || !std::isfinite(duty[1])) {
                supervisor.latch(Fault::Configuration);
                motorEnable(false);
            } else if (after != Fault::None) {
                motorEnable(false);
                routeReady = false;
                supervisor.update(HAL_GetTick(), after, false, false);
            } else if (atFinish(odo.pose)) {
                motorEnable(false);
                supervisor.done();
            } else {
                target[0] = duty[0] * cfg::V_FULL_MM_S;
                target[1] = duty[1] * cfg::V_FULL_MM_S;
                motorSetTargets(target[0], target[1]);
            }
        }
        static State prevState = State::Recovering;
        markRun(supervisor.state() == State::Running);
        if (supervisor.state() == State::Running && supervisor.started())
            logAppend(s.completedAt - supervisor.startedAt(), odo.pose, gNav.lastAlpha(), target, s.tof);
        ledStatusSet(counting ? (s.completedAt / 100) % 2 == 0 : statusLed(supervisor, s.completedAt));
        reportRuntime(supervisor, s);
        traceStep(supervisor, odo.pose, s.completedAt);
        if (prevState == State::Running && supervisor.state() != State::Running) {  // the run stopped (finished, faulted or recovering)
            motorEnable(false);
            noteStall();
        }
        // Save the run record once, when the run has really ended. (A Recovering stop mid-run is not the end: it resumes.)
        if (!gTraceSaved && supervisor.started() && (supervisor.state() == State::Done || supervisor.state() == State::Fault)) {
            gTraceSaved = true;
            saveTraceToFlash();
            noteStall();  // the flash write blocks the loop for a moment; the next step is not a timing fault
        }
        prevState = supervisor.state();
    }
}
