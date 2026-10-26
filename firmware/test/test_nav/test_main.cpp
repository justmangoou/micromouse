/**
 * @file    test_main.cpp
 * @brief   Host tests for control and nav: geometry, odometry, the planner against the Python
 *          planner, and a closed-loop run. Run with `pio test -e native` from firmware/.
 */
#include <unity.h>
#include "../robot_fixture.h"

#include <math.h>

#include <algorithm>

#include "math_util.h"

#include "../../include/map_data.h"
#include "geometry.h"
#include "floodfill.h"
#include "micromouse.h"
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

void test_floodfill_center_and_boundaries() {
    FloodFill maze;
    TEST_ASSERT_EQUAL_UINT16(14, maze.distance({0, 0}));
    TEST_ASSERT_EQUAL_UINT16(0, maze.distance({7, 8}));
    TEST_ASSERT_EQUAL_UINT16(14, maze.distance({15, 15}));
    TEST_ASSERT_FALSE(maze.setPassage({0, 0}, Direction::West, Passage::Open));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Passage::Wall), static_cast<int>(maze.passage({0, 0}, Direction::West)));
    TEST_ASSERT_FALSE(maze.reset(17, 16));
    TEST_ASSERT_EQUAL_INT(16, maze.width());
    TEST_ASSERT_TRUE(maze.reset(3, 5));
    TEST_ASSERT_EQUAL_UINT16(3, maze.distance({0, 0}));
    Cell next{-1, -1};
    Direction dir = Direction::West;
    TEST_ASSERT_TRUE(maze.nextMove({0, 0}, Direction::North, next, dir));
    TEST_ASSERT_EQUAL_INT(0, next.x);
    TEST_ASSERT_EQUAL_INT(1, next.y);
    TEST_ASSERT_FALSE(maze.nextMove({1, 2}, dir, next, dir));
    TEST_ASSERT_FALSE(maze.nextMove({-1, 0}, dir, next, dir));
}

void test_floodfill_wall_updates_and_known_routes() {
    FloodFill maze;
    maze.reset(3, 3);
    Cell goal{2, 0};
    TEST_ASSERT_TRUE(maze.setGoals(&goal, 1));
    TEST_ASSERT_EQUAL_UINT16(2, maze.distance({0, 0}));
    maze.setPassage({0, 0}, Direction::East, Passage::Wall);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Passage::Wall), static_cast<int>(maze.passage({1, 0}, Direction::West)));
    TEST_ASSERT_EQUAL_UINT16(4, maze.distance({0, 0}));
    Cell next{};
    Direction dir = Direction::East;
    TEST_ASSERT_TRUE(maze.nextMove({0, 0}, dir, next, dir));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Direction::North), static_cast<int>(dir));
    maze.setMode(MazeMode::KnownOnly);
    TEST_ASSERT_EQUAL_UINT16(FloodFill::UNREACHABLE, maze.distance({0, 0}));
    TEST_ASSERT_FALSE(maze.nextMove({0, 0}, dir, next, dir));
    maze.setPassage({0, 0}, Direction::North, Passage::Open);
    maze.setPassage({0, 1}, Direction::East, Passage::Open);
    maze.setPassage({1, 1}, Direction::East, Passage::Open);
    maze.setPassage({2, 1}, Direction::South, Passage::Open);
    TEST_ASSERT_EQUAL_UINT16(4, maze.distance({0, 0}));
    maze.setPassage({1, 0}, Direction::West, Passage::Open);
    maze.setPassage({1, 0}, Direction::East, Passage::Open);
    TEST_ASSERT_EQUAL_UINT16(2, maze.distance({0, 0}));
    maze.setPassage({0, 0}, Direction::East, Passage::Unknown);
    TEST_ASSERT_EQUAL_UINT16(4, maze.distance({0, 0}));
    maze.setPassage({0, 0}, Direction::North, Passage::Wall);
    TEST_ASSERT_EQUAL_UINT16(FloodFill::UNREACHABLE, maze.distance({0, 0}));
    Cell invalid{3, 0};
    TEST_ASSERT_FALSE(maze.setGoals(&invalid, 1));
    TEST_ASSERT_TRUE(maze.isGoal(goal));
}

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
    TEST_ASSERT_TRUE(gPath.p[gPath.n - 1].x > 1350.0f && gPath.p[gPath.n - 1].x < 1650.0f);
    for (int i = 0; i < gPath.n; i++) {
        if (gPath.p[i].y < -20.0f) continue;  // below the line is the run-through
        TEST_ASSERT_TRUE(insideTrack(gPath.p[i], -20.0f));
        TEST_ASSERT_TRUE(gPlanner.clearance(gPath.p[i]) >= 40.0f);
    }
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
        finished = truth.y < -90.0f && truth.x > 1250.0f && truth.x < 1750.0f && gNav.progress() > 0.8f;
    }
    TEST_ASSERT_TRUE_MESSAGE(finished, "did not reach the finish line");
    TEST_ASSERT_FALSE_MESSAGE(hit, "the body overlapped a block");
    TEST_ASSERT_TRUE_MESSAGE(t < 40.0f, "too slow");
}

// Ideal range rays against the maze perimeter and a wall blocking the first
// north passage. Exercise gyro-heading turns and cell advancement together.
static void mazeRanges(const math::Pose& pose, float ranges[cfg::TOF_COUNT], int layout) {
    for (int k = 0; k < cfg::TOF_COUNT; ++k) {
        float c = cosf(pose.th), s = sinf(pose.th);
        float ox = pose.x + c * cfg::TOF_LAYOUT.sensors[k].x - s * cfg::TOF_LAYOUT.sensors[k].y;
        float oy = pose.y + s * cfg::TOF_LAYOUT.sensors[k].x + c * cfg::TOF_LAYOUT.sensors[k].y;
        float dx = cosf(pose.th + cfg::TOF_LAYOUT.sensors[k].angle), dy = sinf(pose.th + cfg::TOF_LAYOUT.sensors[k].angle);
        float r = cfg::TOF_MAX_MM;
        for (float x : {0.0f, 540.0f, 180.0f}) {
            if (x == 180 && layout != 2) continue;
            if (fabsf(dx) < 1e-6f) continue;
            float t = (x - ox) / dx, y = oy + t * dy;
            float begin = x == 180 ? 180 : 0, end = x == 180 ? 360 : 540;
            if (t >= 0 && y >= begin && y <= end) r = fminf(r, t);
        }
        for (float y : {0.0f, 540.0f, 180.0f, 360.0f}) {
            if ((y == 180 && layout != 1) || (y == 360 && layout != 2) || fabsf(dy) < 1e-6f) continue;
            float t = (y - oy) / dy, x = ox + t * dx;
            float end = (y == 180 || y == 360) ? 180 : 540;
            if (t >= 0 && x >= 0 && x <= end) r = fminf(r, t);
        }
        ranges[k] = fmaxf(cfg::TOF_MIN_MM, r);
    }
}

void test_micromouse_closed_loop() {
    for (int layout : {0, 1, 2}) { // open, blocked entry, dead end requiring a 180-degree turn
        nav::MicromouseParams p = robotMazeParams();
        p.width = p.height = 3;
        nav::Micromouse mouse(cfg::TOF_LAYOUT, p);
        TEST_ASSERT_TRUE(mouse.start(0));
        math::Pose pose = mouse.startPose();
        float actual[2] = {0, 0};
        bool reliable[cfg::TOF_COUNT] = {true, true, true, true};
        const float dt = 0.033f;
        bool turned = false, backtracked = false;
        for (float t = 0; t < 30; t += dt) {
            float ranges[cfg::TOF_COUNT], command[2];
            mazeRanges(pose, ranges, layout);
            mouse.step(pose, ranges, reliable, t, command);
            turned |= mouse.state() == nav::Micromouse::State::Turning;
            backtracked |= mouse.heading() == nav::Direction::South;
            TEST_ASSERT_NOT_EQUAL(static_cast<int>(nav::Micromouse::State::Fault), static_cast<int>(mouse.state()));
            if (mouse.state() == nav::Micromouse::State::Done) break;
            for (int k = 0; k < 2; ++k) actual[k] += (command[k] - actual[k]) * (1 - expf(-dt / 0.04f));
            float v = (actual[0] + actual[1]) / 2, w = (actual[1] - actual[0]) / p.track;
            math::Pose previous = pose;
            pose.x += v * cosf(pose.th + w * dt / 2) * dt;
            pose.y += v * sinf(pose.th + w * dt / 2) * dt;
            pose.th += w * dt;
            TEST_ASSERT_TRUE(pose.x > 60 && pose.y > 60 && pose.x < 480 && pose.y < 480);
            if (layout == 1 && pose.x < 180) TEST_ASSERT_TRUE(pose.y < 180);
            if (layout == 2 && pose.x < 180) TEST_ASSERT_TRUE(pose.y < 360);
            if (layout == 2 && (previous.x - 180) * (pose.x - 180) < 0) {
                float crossingY = previous.y + (pose.y - previous.y) * (180 - previous.x) / (pose.x - previous.x);
                TEST_ASSERT_TRUE(crossingY < 180 || crossingY > 360);
            }
        }
        TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Done), static_cast<int>(mouse.state()));
        TEST_ASSERT_TRUE(turned);
        TEST_ASSERT_EQUAL_INT(1, mouse.cell().x);
        TEST_ASSERT_EQUAL_INT(1, mouse.cell().y);
        TEST_ASSERT_TRUE(hypotf(pose.x - 270, pose.y - 270) < 10.0f);
        if (layout == 2) TEST_ASSERT_TRUE(backtracked);
        if (layout == 1) TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Passage::Wall),
                                               static_cast<int>(mouse.maze().passage({0, 0}, nav::Direction::North)));
    }
}

void test_micromouse_faults_stop_commands() {
    nav::Micromouse mouse(cfg::TOF_LAYOUT, robotMazeParams());
    TEST_ASSERT_TRUE(mouse.start(0));
    float ranges[cfg::TOF_COUNT] = {300, 300, 300, 300}, command[2];
    bool reliable[cfg::TOF_COUNT] = {true, true, true, true};
    math::Pose pose = mouse.startPose();
    mouse.step(pose, ranges, reliable, 0, command); // verified open, starts advance
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Advancing), static_cast<int>(mouse.state()));
    TEST_ASSERT_EQUAL_INT(0, mouse.cell().y); // not committed before reaching target
    mouse.step(pose, ranges, reliable, 0.033f, command);
    TEST_ASSERT_TRUE(command[0] > 0 && command[1] > 0);
    reliable[1] = false;
    mouse.step(pose, ranges, reliable, 0.066f, command);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Fault), static_cast<int>(mouse.state()));
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
    TEST_ASSERT_EQUAL_FLOAT(0, command[1]);

    mouse.start(0);
    mouse.step(pose, ranges, reliable, 0, command);
    mouse.step(pose, ranges, reliable, 1.1f, command);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Fault), static_cast<int>(mouse.state()));
    reliable[1] = true;
    mouse.start(0);
    mouse.step(pose, ranges, reliable, 0, command);
    mouse.step(pose, ranges, reliable, 6.1f, command); // stalled advancement
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Fault), static_cast<int>(mouse.state()));
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);

    mouse.start(0);
    mouse.step(pose, ranges, reliable, 0, command);
    ranges[1] = ranges[2] = cfg::TOF_MIN_MM; // unexpected obstacle during advance
    mouse.step(pose, ranges, reliable, 0.033f, command);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Micromouse::State::Fault), static_cast<int>(mouse.state()));
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
    TEST_ASSERT_EQUAL_FLOAT(0, command[1]);

    mouse.start(0);
    ranges[1] = ranges[2] = cfg::TOF_MAX_MM; // no-target is not evidence of open space
    mouse.step(pose, ranges, reliable, 0, command);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(nav::Passage::Unknown),
                          static_cast<int>(mouse.maze().passage({0, 0}, nav::Direction::North)));
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_floodfill_center_and_boundaries);
    RUN_TEST(test_floodfill_wall_updates_and_known_routes);
    RUN_TEST(test_geometry);
    RUN_TEST(test_odometry_straight_and_turn);
    RUN_TEST(test_gyro_bias_tracker);
    RUN_TEST(test_speed_controller);
    RUN_TEST(test_speed_controller_deadband);
    RUN_TEST(test_pid_derivative_history);
    RUN_TEST(test_counter_delta_wraps);
    RUN_TEST(test_speed_estimator);
    RUN_TEST(test_planner_matches_python);
    RUN_TEST(test_closed_loop_with_unmapped_blocks);
    RUN_TEST(test_micromouse_closed_loop);
    RUN_TEST(test_micromouse_faults_stop_commands);
    return UNITY_END();
}
