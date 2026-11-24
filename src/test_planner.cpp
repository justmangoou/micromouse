/**
 * @file    test_planner.cpp
 * @brief   Timing and stack check of the low-memory course planner on the Blue Pill (env:test_planner). No sensors, no
 *          motors. Loads the stored map, plans the route, plans again from another pose, then feeds a fake obstacle
 *          to the navigator so it replans around it. Results are in gPlanTest for ST-Link: milliseconds per
 *          step, path length, and how many bytes of stack were never touched.
 */
#include "board.h"
#include "config.h"
#include "map_data.h"
#include "navigator.h"

extern "C" uint32_t _end;
extern "C" uint32_t navTicks() { return DWT->CYCCNT; }
extern volatile uint32_t gNavProfile[12];  // end of .bss, from the linker script: the stack grows down toward it

struct PlanTestStatus {
    uint32_t stage;           // 1 init, 2 start plan, 3 reposition, 4 obstacle replans, 5 done
    uint32_t initMs;          // setMap: the clearance field
    uint32_t startMs;         // first plan
    uint32_t repositionMs;    // plan from another pose
    uint32_t maxStepMs;       // slowest navigator step while the obstacle is detected (includes the replan)
    uint32_t okStart, okReposition;
    uint32_t pathPoints;
    float pathLengthMm;
    uint32_t extras;          // unmapped blocks the navigator found
    uint32_t replans;
    uint32_t stackFreeBytes;  // never touched
    uint32_t heapEnd;
    uint32_t plainStepMs;     // slowest step that did not replan
};
volatile PlanTestStatus gPlanTest = {};

namespace {

nav::NavParams params() {
    nav::NavParams p;
    p.vFull = cfg::V_FULL_MM_S;
    p.track = cfg::WHEEL_TRACK_MM;
    return p;
}

nav::Navigator gNav(cfg::TOF_LAYOUT, params());

constexpr uint32_t PAINT = 0xA5A5A5A5u;

void paintStack() {
    uint32_t* p = &_end;
    uint32_t* sp = reinterpret_cast<uint32_t*>(__get_MSP()) - 16;
    while (p < sp) *p++ = PAINT;
}

uint32_t untouchedStack() {
    uint32_t* p = &_end;
    uint32_t n = 0;
    while (*p == PAINT && p < reinterpret_cast<uint32_t*>(__get_MSP())) {
        ++p;
        n += 4;
    }
    return n;
}

}  // namespace

int main() {
    boardInit();
    paintStack();

    gPlanTest.stage = 1;
    uint32_t t0 = HAL_GetTick();
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    gPlanTest.initMs = HAL_GetTick() - t0;

    gPlanTest.stage = 2;
    t0 = HAL_GetTick();
    gPlanTest.okStart = gNav.start({300.0f, -80.0f, 1.5708f}, 0.0f);
    gPlanTest.startMs = HAL_GetTick() - t0;
    gPlanTest.pathPoints = gNav.path().n;
    gPlanTest.pathLengthMm = gNav.path().n > 0 ? gNav.path().s[gNav.path().n - 1] : 0.0f;

    // Poses taken from the planned route, so they are on the track and clear of the mapped blocks.
    const int k0 = 8;
    const nav::Path& route = gNav.path();
    math::Pose pose = {route.p[k0].x, route.p[k0].y,
                       atan2f(route.p[k0 + 1].y - route.p[k0].y, route.p[k0 + 1].x - route.p[k0].x)};

    gPlanTest.stage = 3;
    t0 = HAL_GetTick();
    gPlanTest.okReposition = gNav.reposition(pose, 1.0f);
    gPlanTest.repositionMs = HAL_GetTick() - t0;

    // An obstacle the map does not have, 80 mm ahead of the front sensor, for 40 control steps.
    gPlanTest.stage = 4;
    float tof[cfg::TOF_COUNT];
    for (int i = 0; i < cfg::TOF_COUNT; ++i) tof[i] = cfg::TOF_MAX_MM;
    tof[cfg::TOF_FRONT] = 80.0f;
    uint32_t maxStep = 0;
    for (int k = 0; k < 40; ++k) {
        float duty[2];
        int before = gNav.replans();
        t0 = HAL_GetTick();
        gNav.step(pose, 0.0f, tof, 2.0f + 0.033f * k, duty);
        uint32_t dt = HAL_GetTick() - t0;
        if (dt > maxStep) maxStep = dt;
        gPlanTest.maxStepMs = maxStep;
        if (gNav.replans() == before && dt > gPlanTest.plainStepMs) gPlanTest.plainStepMs = dt;
    }
    gPlanTest.extras = gNav.unmappedBlocks();
    gPlanTest.replans = gNav.replans();
    gPlanTest.stackFreeBytes = untouchedStack();
    gPlanTest.stage = 5;
    while (true) {
        ledStatusSet((HAL_GetTick() / 250) % 2 == 0);
        HAL_Delay(10);
    }
}
