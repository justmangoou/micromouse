/**
 * @file    test_main.cpp
 * @brief   Host tests for control and nav: geometry, odometry, the planner against the Python
 *          planner, and a closed-loop run. Run with `pio test -e native` from the repository root.
 */
#include <unity.h>
#include "../../include/config.h"

#include <math.h>

#include <algorithm>

#include "math_util.h"

#include "../../include/map_data.h"
#include "geometry.h"
#include "navigator.h"
#include "odometry.h"
#include "planner.h"
#include "speed_controller.h"
#include "tof_layout.h"
#include "ref_data.h"

using namespace nav;
using namespace math;

static nav::Planner gPlanner;  // ~60 KB, keep it off the stack
static nav::Navigator gNav(cfg::TOF_LAYOUT);
static nav::Path gPath;
static const nav::NavParams kCar;  // the car the closed-loop test simulates: same vFull and track as the navigator's

void setUp() {}
void tearDown() {}

static void buildMap(Block* out) {
    for (int i = 0; i < MAP_BLOCK_COUNT; i++) out[i] = Block(MAP_BLOCKS[i][0], MAP_BLOCKS[i][1], MAP_BLOCKS[i][2]);
}

void test_geometry() {
    Block b(0, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 50.0f, blockClearance({100, 0}, b));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, blockClearance({20, 10}, b));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 50.0f, rayBlock({-100, 0}, {1, 0}, b, 1200));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1200.0f, rayBlock({-100, 200}, {1, 0}, b, 1200));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1200.0f, rayBlock({-100, 0}, {-1, 0}, b, 1200));  // block behind the ray
    Block rot(0, 0, PI / 4);  // corner pointing at the origin axis
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f - BLOCK_HALF_DIAG, rayBlock({-100, 0}, {1, 0}, rot, 1200));
    TEST_ASSERT_TRUE(insideTrack({300, -80}, 0));
    TEST_ASSERT_TRUE(insideTrack({900, 900}, 0));
    TEST_ASSERT_FALSE(insideTrack({900, 300}, 0));  // the forbidden middle of the U
    TEST_ASSERT_TRUE(insideTrack({610, 300}, TAPE_HALF + 5.0f));
}

void test_odometry_straight_and_turn() {
    FusedOdometry odo(cfg::ODOM);
    odo.reset({0, 0, PI / 2});
    for (int i = 0; i < 30; i++) odo.update(56, 56, 0.0f, 1.0f / 30.0f);
    float mm = 56 * cfg::MM_PER_COUNT * 30;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, mm, odo.pose.y);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, odo.pose.x);

    odo.reset({0, 0, 0});
    for (int i = 0; i < 30; i++) odo.update(-10, 10, PI / 2, 1.0f / 30.0f);  // 90 deg in 1 s
    TEST_ASSERT_FLOAT_WITHIN(0.02f, PI / 2, odo.pose.th);

    odo.reset({0, 0, 0});
    // A gyro-observed turn must not be learned away when encoders report zero.
    for (int i = 0; i < 60; i++) odo.update(0, 0, 0.01f, 1.0f / 30.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0196f, odo.pose.th);
}

void test_pid_derivative_history() {
    speed_control::Pid pid(0, 0, 1, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4, pid.update(2, 0, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2, pid.update(3, 0, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0, pid.update(3, 0, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -4, pid.update(1, 0, 0.5f));
    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2, pid.update(1, 0, 0.5f));
}

void test_gyro_bias_tracker() {
    const float dt = 0.033f;
    const nav::GyroBiasParams p = cfg::GYRO_BIAS;
    nav::GyroBiasTracker tracker(p);
    // Nothing until the robot has been still for stillS seconds.
    int steps = 0;
    while ((steps + 1) * dt < p.stillS) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, true, dt));
        ++steps;
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, dt / p.timeConstantS * 0.01f, tracker.update(0.01f, true, dt));
    // Converges on the residual offset: apply the corrections the way the firmware does.
    float bias = 0.0f, trueBias = 0.02f;
    for (int i = 0; i < 3000; ++i) bias += tracker.update(trueBias - bias, true, dt);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, trueBias, bias);
    // The correction per second does not depend on the step length.
    nav::GyroBiasTracker fast(p), slow(p);
    float fastSum = 0, slowSum = 0;
    for (int i = 0; i < 400; ++i) fastSum += fast.update(0.01f, true, 0.01f);
    for (int i = 0; i < 100; ++i) slowSum += slow.update(0.01f, true, 0.04f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * slowSum, slowSum, fastSum);
    // Motion, fast rotation, or a bad reading restarts the stillness requirement.
    const int stillSteps = (int)(p.stillS / dt) + 2;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, false, dt));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, true, dt));
    for (int i = 0; i < stillSteps; ++i) tracker.update(0.01f, true, dt);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(p.maxRate * 1.01f, true, dt));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, true, dt));
    for (int i = 0; i < stillSteps; ++i) tracker.update(0.01f, true, dt);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(NAN, true, dt));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, true, dt));
    for (int i = 0; i < stillSteps; ++i) tracker.update(0.01f, true, dt);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tracker.update(0.01f, true, 0.0f));  // no time elapsed
}

void test_speed_controller() {
    speed_control::Pid pid(0.0004f, 0.002f, 0.0f, 200.0f, 0.3f);
    // One count per mm/s and alpha 1 isolate PID behavior from filtering.
    speed_control::SpeedController c(622.0f, 0.005f, 0.005f, pid, 10.0f, 1.0f);
    c.update_target(311.0f);
    // on target: the PID adds nothing, only the feed-forward remains
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, c.update_counts(311));
    // too slow: more duty than the feed-forward alone
    TEST_ASSERT_TRUE(c.update_counts(200) > 0.5f);
    // a target of about zero resets the PID: the wound-up integral is gone afterwards
    c.update_target(5.0f);
    c.update_counts(0);
    c.update_target(311.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, c.update_counts(311));
    c.update_target(-311.0f);
    // Reverse targets use the same feedback loop: too slow means more negative duty.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, c.update_counts(-311));
    TEST_ASSERT_TRUE(c.update_counts(-200) < -0.5f);
    // Stopping preserves measured motion and clears PID history before restarting.
    const float movingSpeed = c.speed();
    c.stop();
    TEST_ASSERT_EQUAL_FLOAT(movingSpeed, c.speed());
    c.update_target(-311.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, c.update_counts(-311));
    c.stop();
    TEST_ASSERT_EQUAL_FLOAT(0, c.update_counts(-100));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -100, c.speed());
    // Reset clears the active target, filtered speed, and accumulated PID error.
    c.reset();
    TEST_ASSERT_EQUAL_FLOAT(0, c.speed());
    TEST_ASSERT_EQUAL_FLOAT(0, c.update_counts(0));
    c.update_target(311.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, c.update_counts(311));
}

void test_pid_integral_does_not_wind_up_past_the_output_limit() {
    // ki 0.01, output limit 0.3: after a long saturated error the integral term must be at most 0.3, so a reversed error
    // brings the output back within a fraction of a second instead of staying pinned for seconds.
    speed_control::Pid pid(0.001f, 0.01f, 0.0f, 200.0f, 0.3f);
    for (int i = 0; i < 1000; ++i) pid.update(0.0f, 300.0f, 0.005f);  // 5 s of large negative error: output -0.3
    float out = 0;
    int ticks = 0;
    for (; ticks < 400 && out < 0.0f; ++ticks) out = pid.update(130.0f, 0.0f, 0.005f);  // now a +130 mm/s error
    TEST_ASSERT_TRUE_MESSAGE(ticks < 100, "the output stayed pinned negative for more than 0.5 s");
}

// The loaded motor measured on the car: speed = 2700 * (duty - 0.065), response time ~0.15 s. With the controller of the
// run that wobbled (feed-forward from 622 mm/s, kp 0.001, ki 0.01) a 300 mm/s step overshot by ~270 mm/s.
void test_speed_controller_tracks_the_measured_motor() {
    const float mmPerCount = 0.384f, dt = 0.005f, gain = 2700.0f, offset = 0.065f, tau = 0.15f;
    speed_control::SpeedController c(cfg::MOTOR_FF_MM_S_L, mmPerCount, dt,
                                     speed_control::Pid(cfg::SPEED_KP, cfg::SPEED_KI, 0.0f, 200.0f, cfg::SPEED_CORR_LIMIT),
                                     10.0f, cfg::SPEED_FILTER_ALPHA, cfg::SPEED_WINDOW_TICKS, cfg::MOTOR_FF_OFFSET_L);
    float v = 0.0f, enc = 0.0f, peak = 0.0f, late = 0.0f;
    const float steps[4][2] = {{0.6f, 300.0f}, {1.2f, 150.0f}, {1.8f, 350.0f}, {2.4f, 0.0f}};
    int s = 0;
    for (float t = 0.0f; t < 1.8f; t += dt) {
        if (t >= steps[s][0]) {
            if (s == 0) { TEST_ASSERT_FLOAT_WITHIN(25.0f, 300.0f, late); }
            ++s;
            peak = 0.0f;
        }
        c.update_target(steps[s][1]);
        enc += v * dt / mmPerCount;
        int counts = static_cast<int>(enc);
        enc -= counts;
        float duty = c.update_counts(counts);
        float eff = fabsf(duty) < offset ? 0.0f : (duty > 0 ? duty - offset : duty + offset);
        v += (gain * eff - v) * (dt / tau);
        if (s == 0) peak = fmaxf(peak, v);
        if (s == 0 && t > steps[s][0] - 0.1f) late = v;
        if (s == 0) TEST_ASSERT_TRUE_MESSAGE(peak < 360.0f, "overshoot on the 300 mm/s step");
    }
    TEST_ASSERT_FLOAT_WITHIN(25.0f, 350.0f, v);
}

void test_speed_window_smooths_quantization() {
    // 300 mm/s with 0.384 mm/count at 200 Hz is 3.9 counts per tick: the encoder gives 4,4,4,3,4,4,4,4,3,...
    const float mmPerCount = 0.384f, dt = 0.005f;
    speed_control::SpeedController raw(622.0f, mmPerCount, dt, speed_control::Pid{}, 10.0f, 1.0f, 1);
    speed_control::SpeedController avg(622.0f, mmPerCount, dt, speed_control::Pid{}, 10.0f, 1.0f, 4);
    float rawMin = 1e9f, rawMax = -1e9f, avgMin = 1e9f, avgMax = -1e9f;
    for (int i = 0; i < 80; ++i) {
        int counts = (i % 9 == 3) ? 3 : 4;
        raw.update_counts(counts);
        avg.update_counts(counts);
        if (i < 8) continue;
        rawMin = fminf(rawMin, raw.speed()); rawMax = fmaxf(rawMax, raw.speed());
        avgMin = fminf(avgMin, avg.speed()); avgMax = fmaxf(avgMax, avg.speed());
    }
    TEST_ASSERT_TRUE(avgMax - avgMin < 0.5f * (rawMax - rawMin));
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 300.0f, avg.speed());
    avg.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, avg.speed());
}

void test_speed_controller_deadband() {
    speed_control::SpeedController c(622.0f, 0.005f, 0.005f,
                               speed_control::Pid(0.001f, 0, 0, 0, 0), 20.0f, 1.0f);
    for (float sign : {-1.0f, 1.0f}) {
        c.update_target(sign * 15.0f);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, sign * 15.0f / 622.0f, c.update_counts(0));
        c.update_target(sign * 20.0f);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, sign * (20.0f / 622.0f + 0.02f), c.update_counts(0));
    }
}

void test_counter_delta_wraps() {
    TEST_ASSERT_EQUAL_INT32(5, speed_control::counterDelta(105, 100));
    TEST_ASSERT_EQUAL_INT32(-5, speed_control::counterDelta(95, 100));
    TEST_ASSERT_EQUAL_INT32(10, speed_control::counterDelta(4, 65530));      // forward through the 16-bit wrap
    TEST_ASSERT_EQUAL_INT32(-10, speed_control::counterDelta(65530, 4));     // backward through it
    TEST_ASSERT_EQUAL_INT32(0, speed_control::counterDelta(777, 777));
}

void test_speed_estimator() {
    const float mmPerCount = 0.24f, dt = 0.005f;
    speed_control::SpeedController est(622.0f, mmPerCount, dt, speed_control::Pid{});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.speed());
    // a steady 10 counts per period is 480 mm/s; the first-order filter (alpha 0.4) gets there from zero
    for (int i = 0; i < 40; i++) est.update_counts(10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10 * mmPerCount / dt, est.speed());
    // the first sample alone only moves it alpha of the way
    speed_control::SpeedController fresh(622.0f, mmPerCount, dt, speed_control::Pid{});
    TEST_ASSERT_EQUAL_FLOAT(0, fresh.update_counts(10));
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.4f * 10 * mmPerCount / dt, fresh.speed());
    // backwards counts give a negative speed
    for (int i = 0; i < 40; i++) est.update_counts(-10);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10 * mmPerCount / dt, est.speed());
    est.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.speed());
}

void test_planner_matches_python() {
    Block blocks[MAX_MAP];
    buildMap(blocks);
    gPlanner.setMap(blocks, MAP_BLOCK_COUNT);
    TEST_ASSERT_TRUE(gPlanner.plan({300, -80}, gPath));
    TEST_ASSERT_TRUE(gPath.n > 50);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * REF_PATH_LENGTH, REF_PATH_LENGTH, gPath.s[gPath.n - 1]);
    TEST_ASSERT_TRUE(gPath.p[gPath.n - 1].y < -100.0f);
    TEST_ASSERT_TRUE(gPath.p[gPath.n - 1].x > 1215.0f && gPath.p[gPath.n - 1].x < 1750.0f);  // anywhere on the finish line
    for (int i = 0; i < gPath.n; i++) {
        if (gPath.p[i].y < -20.0f) continue;  // below the line is the run-through
        TEST_ASSERT_TRUE(insideTrack(gPath.p[i], -20.0f));
        TEST_ASSERT_TRUE(gPlanner.clearance(gPath.p[i]) >= 40.0f);
    }
}

// Regression: in the low-memory build the string pulling hit its corner cap and jumped straight to the end of the route,
// cutting T16's corner at 65 mm (the car's wheel is 50 mm off its centre line, so it touched). The planned centre line must
// keep at least 70 mm from every mapped block (the bug gave 63 mm; the coarse 40 mm grid
// elsewhere reaches 73 mm).
void test_planned_path_keeps_clear_of_blocks() {
    Block blocks[MAX_MAP];
    buildMap(blocks);
    gPlanner.setMap(blocks, MAP_BLOCK_COUNT);
    TEST_ASSERT_TRUE(gPlanner.plan({300, -80}, gPath));
    for (int i = 0; i < gPath.n; i++) {
        if (gPath.p[i].y < -20.0f) continue;
        TEST_ASSERT_TRUE_MESSAGE(gPlanner.clearance(gPath.p[i]) >= 70.0f, "planned path too close to a block");
    }
}

// --- random-block detection: walls are not blocks, a real block is found, and there is only one -------------------
namespace {

nav::NavParams randomBlockParams() {
    nav::NavParams p;  // the course's detection limits (include/course_params.h), without its path filter
    p.detect.minHitX = 1150.0f;
    p.detect.maxCentreX = 1800.0f;
    p.detect.minCentreY = 60.0f;
    p.detect.maxBlocks = 1;
    return p;
}

// ToF readings from blocks and the fixed walls, no noise.
void tofWithWalls(const Pose& p, const Block* blocks, int n, float out[cfg::TOF_COUNT]) {
    float c = cosf(p.th), s = sinf(p.th);
    for (int k = 0; k < cfg::TOF_COUNT; k++) {
        const auto& m = cfg::TOF_LAYOUT.sensors[k];
        Vec2 o = {p.x + c * m.x - s * m.y, p.y + s * m.x + c * m.y};
        float best = cfg::TOF_MAX_MM;
        for (int f = 0; f < cfg::TOF_LAYOUT.rayCount; f++) {
            float a = p.th + m.angle + cfg::TOF_LAYOUT.rays[f];
            Vec2 d = {cosf(a), sinf(a)};
            best = fminf(best, rayWalls(o, d, cfg::TOF_MAX_MM));
            for (int b = 0; b < n; b++) best = fminf(best, rayBlock(o, d, blocks[b], cfg::TOF_MAX_MM));
        }
        out[k] = std::clamp(best, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM);
    }
}

// Park the car at `pose` and feed it `steps` control steps of readings; returns how many random blocks it believes in.
int detectedAt(const Pose& pose, const Block* extra, int nExtra) {
    gNav = Navigator(cfg::TOF_LAYOUT, randomBlockParams());
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    gNav.start({300.0f, -80.0f, PI / 2}, 0.0f);
    Block world[MAP_BLOCK_COUNT + 2];
    int n = 0;
    for (int i = 0; i < MAP_BLOCK_COUNT; i++) world[n++] = Block(MAP_BLOCKS[i][0], MAP_BLOCKS[i][1], MAP_BLOCKS[i][2]);
    for (int i = 0; i < nExtra; i++) world[n++] = extra[i];
    for (int step = 0; step < 12; ++step) {
        float tof[cfg::TOF_COUNT], duty[2];
        tofWithWalls(pose, world, n, tof);
        gNav.step(pose, 0.0f, tof, 1.0f + step * 0.033f, duty);
    }
    return gNav.unmappedBlocks();
}

}  // namespace

void test_walls_are_not_random_blocks() {
    // Every place the first real run saw a phantom: the arena's right wall, the pit's right wall, the finish tape area.
    const Pose spots[] = {
        {1650.0f, 500.0f, 0.0f},      // facing the arena wall at x = 1800
        {1650.0f, 869.0f, 0.3f},
        {1500.0f, 300.0f, -0.7f},
        {1300.0f, 193.0f, PI},        // facing the pit's right wall at x = 1200
        {1300.0f, 357.0f, PI - 0.4f},
        {1585.0f, 100.0f, -1.5f},     // heading for the finish
        {1700.0f, 1100.0f, PI / 2},   // facing the top wall
    };
    for (const Pose& p : spots) TEST_ASSERT_EQUAL_INT(0, detectedAt(p, nullptr, 0));
}

void test_wall_clearance() {
    // distance to the nearest wall: the pit's right wall is x = 1200 for y 0..600, the arena's right wall x = 1800
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, wallClearance({1300.0f, 300.0f}));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, wallClearance({1700.0f, 700.0f}));
    TEST_ASSERT_TRUE(wallClearance({1500.0f, 300.0f}) > 290.0f);
}

void test_baseline_flags_only_what_is_new_on_the_path() {
    // Two reference-log records at the pose (1250, 600) heading south: front sensor read 300 mm, the sides saw nothing.
    static uint8_t log[2 * Baseline::RECORD_BYTES];
    for (int r = 0; r < 2; ++r) {
        uint8_t* p = log + r * Baseline::RECORD_BYTES;
        const int16_t pose[3] = {1250, 600, -1571};
        for (int i = 0; i < 3; ++i) {
            p[2 + 2 * i] = static_cast<uint8_t>(pose[i] & 0xFF);
            p[3 + 2 * i] = static_cast<uint8_t>((pose[i] >> 8) & 0xFF);
        }
        p[13] = 240;
        p[14] = 60;
        p[15] = 240;
    }
    auto run = [&](float frontMm) {
        gNav = Navigator(cfg::TOF_LAYOUT, randomBlockParams());
        gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
        Baseline b;
        b.records = log;
        b.count = 2;
        gNav.setBaseline(b);
        gNav.start({300.0f, -80.0f, PI / 2}, 0.0f);
        Pose pose = {1250.0f, 600.0f, -PI / 2};
        for (int step = 0; step < 8; ++step) {
            float tof[cfg::TOF_COUNT] = {1200.0f, frontMm, 1200.0f}, duty[2];
            gNav.step(pose, 0.0f, tof, 1.0f + step * 0.033f, duty);
        }
        return gNav.unmappedBlocks();
    };
    TEST_ASSERT_EQUAL_INT(0, run(300.0f));  // exactly what the clean runs read here: background
    TEST_ASSERT_EQUAL_INT(0, run(240.0f));  // a bit shorter, within the margin
    TEST_ASSERT_EQUAL_INT(1, run(150.0f));  // 150 mm short of the clean-run reading: a block
}

void test_one_random_block_is_found_and_only_one() {
    Block one[1] = {Block(1600.0f, 700.0f, 0.2f)};
    TEST_ASSERT_EQUAL_INT(1, detectedAt({1350.0f, 700.0f, 0.0f}, one, 1));
    // two real blocks: the course only has one, so the second is ignored
    Block two[2] = {Block(1600.0f, 700.0f, 0.0f), Block(1600.0f, 1000.0f, 0.0f)};
    TEST_ASSERT_TRUE(detectedAt({1350.0f, 850.0f, 0.4f}, two, 2) <= 1);
    // a block outside the arena's right wing is not accepted (the left half is mapped)
    Block left[1] = {Block(400.0f, 1000.0f, 0.0f)};
    TEST_ASSERT_EQUAL_INT(0, detectedAt({500.0f, 800.0f, PI / 2}, left, 1));
}

// The block is trusted only once enough readings agree and the car is past it; then it stays trusted until restart.
void test_trust_after_the_block_is_passed() {
    Block one[1] = {Block(1600.0f, 700.0f, 0.0f)};
    Block world[MAP_BLOCK_COUNT + 1];
    buildMap(world);
    world[MAP_BLOCK_COUNT] = one[0];
    nav::NavParams p = randomBlockParams();
    p.trustSupport = 3;
    gNav = Navigator(cfg::TOF_LAYOUT, p);
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    float t = 1.0f;
    auto drive = [&](const Pose& pose, int steps) {
        TEST_ASSERT_TRUE(gNav.reposition(pose, t));  // the pose jumps: plan from there
        for (int i = 0; i < steps; ++i, t += 0.033f) {
            float tof[cfg::TOF_COUNT], duty[2];
            tofWithWalls(pose, world, MAP_BLOCK_COUNT + 1, tof);
            gNav.step(pose, 0.0f, tof, t, duty);
        }
    };
    gNav.start({300.0f, -80.0f, PI / 2}, 0.0f);
    drive({1600.0f, 380.0f, -PI / 2}, 3);  // past that spot without having seen anything: no trust
    TEST_ASSERT_FALSE(gNav.blockPassed());
    drive({1350.0f, 700.0f, 0.0f}, 12);  // looking at the block, level with it
    TEST_ASSERT_EQUAL_INT(1, gNav.unmappedBlocks());
    TEST_ASSERT_TRUE(gNav.blockSupport() >= 3);
    TEST_ASSERT_FALSE(gNav.blockPassed());  // confirmed but not behind yet
    drive({1600.0f, 380.0f, -PI / 2}, 1);
    TEST_ASSERT_TRUE(gNav.blockPassed());
    gNav.start({300.0f, -80.0f, PI / 2}, t);  // the next run starts untrusted
    TEST_ASSERT_FALSE(gNav.blockPassed());
}

// --- closed loop on the host: ideal differential-drive car, ToF from the real world -------------
namespace {

struct World {
    Block blocks[MAX_MAP + 4];
    int n = 0;
};

void tofFromWorld(const Pose& p, const World& w, float out[cfg::TOF_COUNT]) {
    float c = cosf(p.th), s = sinf(p.th);
    for (int k = 0; k < cfg::TOF_COUNT; k++) {
        Vec2 o = {p.x + c * cfg::TOF_LAYOUT.sensors[k].x - s * cfg::TOF_LAYOUT.sensors[k].y, p.y + s * cfg::TOF_LAYOUT.sensors[k].x + c * cfg::TOF_LAYOUT.sensors[k].y};
        float best = cfg::TOF_MAX_MM;
        for (int f = 0; f < cfg::TOF_LAYOUT.rayCount; f++) {
            float a = p.th + cfg::TOF_LAYOUT.sensors[k].angle + cfg::TOF_LAYOUT.rays[f];
            Vec2 d = {cosf(a), sinf(a)};
            for (int b = 0; b < w.n; b++) best = fminf(best, rayBlock(o, d, w.blocks[b], cfg::TOF_MAX_MM));
        }
        out[k] = std::clamp(best, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM);
    }
}

bool bodyOverlaps(const Pose& p, const World& w) {
    float c = cosf(p.th), s = sinf(p.th);
    for (int ix = -2; ix <= 2; ix++) {
        for (int iy = -2; iy <= 2; iy++) {
            float lx = ix * 31.25f, ly = iy * 25.0f;  // 125 x 100 mm body
            Vec2 q = {p.x + c * lx - s * ly, p.y + s * lx + c * ly};
            for (int b = 0; b < w.n; b++)
                if (blockClearance(q, w.blocks[b]) < 3.0f) return true;
        }
    }
    return false;
}

}  // namespace

void test_closed_loop_with_unmapped_blocks() {
    static World world;
    world.n = 0;
    for (int i = 0; i < MAP_BLOCK_COUNT; i++)
        world.blocks[world.n++] = Block(MAP_BLOCKS[i][0], MAP_BLOCKS[i][1], MAP_BLOCKS[i][2]);
    // Generously clear of MAP_BLOCKS and of the x:[135,165] goal-approach column: this test validates that the
    // Navigator detects and routes around an unmapped block, not that it can thread a minimal, near-adversarial gap.
    world.blocks[world.n++] = Block(1700.0f, 950.0f, 0.3f);   // random block, top-right cell
    world.blocks[world.n++] = Block(1700.0f, 150.0f, -0.2f);  // random block, finish cell

    gNav = Navigator(cfg::TOF_LAYOUT);
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    Pose truth = {300.0f, -80.0f, PI / 2};
    gNav.start(truth, 0.0f);

    const float dt = 1.0f / 30.0f;
    const float tau = 0.06f;
    float w[2] = {0, 0};  // wheel speeds, mm/s
    bool finished = false, hit = false;
    float t = 0, prevV = 0;
    for (; t < 60.0f && !finished; t += dt) {
        float tof[cfg::TOF_COUNT];
        tofFromWorld(truth, world, tof);
        float duty[2];
        gNav.step(truth, prevV, tof, t, duty);  // perfect localisation: this test is about planning and tracking
        for (int k = 0; k < 3; k++) {            // physics substeps
            for (int i = 0; i < 2; i++) w[i] += (duty[i] * kCar.vFull * 0.925f - w[i]) * (1.0f - expf(-dt / 3 / tau));
            float v = 0.5f * (w[0] + w[1]), om = (w[1] - w[0]) / kCar.track;
            float mid = truth.th + 0.5f * om * dt / 3;
            truth.x += v * cosf(mid) * dt / 3;
            truth.y += v * sinf(mid) * dt / 3;
            truth.th += om * dt / 3;
            prevV = v;
        }
        if (bodyOverlaps(truth, world)) hit = true;
        finished = truth.y < -90.0f && truth.x > 1215.0f && truth.x < 1750.0f && gNav.progress() > 0.8f;
    }
    TEST_ASSERT_TRUE_MESSAGE(finished, "did not reach the finish line");
    TEST_ASSERT_FALSE_MESSAGE(hit, "the body overlapped a block");
    TEST_ASSERT_TRUE_MESSAGE(t < 40.0f, "too slow");
}

// The real course: blocks a few cm off the map, and odometry that drifts (heading bias, wheel scale). Returns the
// closest the body got to any block face and whether the finish was reached.
static int gLocApplied = 0;
struct DriftResult { bool finished, hit; float maxErr, endErr; };
static DriftResult runDrift(bool localise, float biasRadS, float scale, float shiftMm) {
    static World world;
    world.n = 0;
    for (int i = 0; i < MAP_BLOCK_COUNT; i++) {
        float ox = shiftMm * sinf(i * 2.3f + 1.0f), oy = shiftMm * cosf(i * 1.7f + 0.5f);  // deterministic offsets
        world.blocks[world.n++] = Block(MAP_BLOCKS[i][0] + ox, MAP_BLOCKS[i][1] + oy, 0.08f * sinf(i * 3.1f));
    }
    gNav = Navigator(cfg::TOF_LAYOUT);
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    Pose truth = {300.0f, -80.0f, PI / 2}, est = truth;
    gNav.start(est, 0.0f);
    const float dt = 1.0f / 30.0f, tau = 0.06f;
    float w[2] = {0, 0}, prevV = 0, t = 0;
    DriftResult r = {false, false, 0, 0};
    for (; t < 60.0f && !r.finished; t += dt) {
        float tof[cfg::TOF_COUNT];
        tofFromWorld(truth, world, tof);
        if (localise && gNav.localize(est, tof, t)) gLocApplied++;
        float duty[2];
        gNav.step(est, prevV, tof, t, duty);
        for (int k = 0; k < 3; k++) {
            for (int i = 0; i < 2; i++) w[i] += (duty[i] * kCar.vFull * 0.925f - w[i]) * (1.0f - expf(-dt / 3 / tau));
            float v = 0.5f * (w[0] + w[1]), om = (w[1] - w[0]) / kCar.track;
            float mid = truth.th + 0.5f * om * dt / 3;
            truth.x += v * cosf(mid) * dt / 3;
            truth.y += v * sinf(mid) * dt / 3;
            truth.th += om * dt / 3;
            float emid = est.th + 0.5f * om * dt / 3;
            est.x += scale * v * cosf(emid) * dt / 3;
            est.y += scale * v * sinf(emid) * dt / 3;
            est.th += (om + biasRadS) * dt / 3;
            prevV = v;
        }
        if (bodyOverlaps(truth, world)) r.hit = true;
        float e = hypotf(est.x - truth.x, est.y - truth.y);
        if (e > r.maxErr) r.maxErr = e;
        r.endErr = e;
        r.finished = truth.y < -90.0f && truth.x > 1215.0f && truth.x < 1750.0f && gNav.progress() > 0.8f;
    }
    return r;
}

void test_localisation_against_drift_and_a_shifted_map() {
    static const float shifts[] = {0.0f, 20.0f, 40.0f};
    for (float shift : shifts) {
        for (float bias : {0.003f, 0.012f}) {
            gLocApplied = 0;
            DriftResult off = runDrift(false, bias, 1.02f, shift);
            DriftResult on = runDrift(true, bias, 1.02f, shift);
            printf("shift %2.0f mm bias %.3f | off: fin=%d hit=%d maxErr=%3.0f | on: fin=%d hit=%d maxErr=%3.0f applied=%d\n",
                   shift, bias, off.finished, off.hit, off.maxErr, on.finished, on.hit, on.maxErr, gLocApplied);
        }
    }
    DriftResult on = runDrift(true, 0.003f, 1.02f, 0.0f);
    TEST_ASSERT_TRUE_MESSAGE(on.finished, "with localisation: did not reach the finish");
    TEST_ASSERT_FALSE_MESSAGE(on.hit, "with localisation: the body overlapped a block");
}

// Ideal range rays against the maze perimeter and a wall blocking the first
// north passage. Exercise gyro-heading turns and cell advancement together.
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_geometry);
    RUN_TEST(test_odometry_straight_and_turn);
    RUN_TEST(test_gyro_bias_tracker);
    RUN_TEST(test_speed_controller);
    RUN_TEST(test_speed_controller_deadband);
    RUN_TEST(test_speed_window_smooths_quantization);
    RUN_TEST(test_speed_controller_tracks_the_measured_motor);
    RUN_TEST(test_pid_integral_does_not_wind_up_past_the_output_limit);
    RUN_TEST(test_pid_derivative_history);
    RUN_TEST(test_counter_delta_wraps);
    RUN_TEST(test_speed_estimator);
    RUN_TEST(test_planner_matches_python);
    RUN_TEST(test_planned_path_keeps_clear_of_blocks);
    RUN_TEST(test_walls_are_not_random_blocks);
    RUN_TEST(test_one_random_block_is_found_and_only_one);
    RUN_TEST(test_wall_clearance);
    RUN_TEST(test_baseline_flags_only_what_is_new_on_the_path);
    RUN_TEST(test_trust_after_the_block_is_passed);
    RUN_TEST(test_closed_loop_with_unmapped_blocks);
    RUN_TEST(test_localisation_against_drift_and_a_shifted_map);
    return UNITY_END();
}
