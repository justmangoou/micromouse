/**
 * @file    test_main.cpp
 * @brief   Host tests for math and nav: geometry, odometry, the planner against the Python
 *          planner, and a closed-loop run. Run with `pio test -e native` from firmware/.
 */
#include <unity.h>

#include <math.h>

#include <algorithm>

#include "../../include/map_data.h"
#include "geometry.h"
#include "floodfill.h"
#include "navigator.h"
#include "odometry.h"
#include "planner.h"
#include "speed_controller.h"
#include "speed_estimator.h"
#include "tof_layout.h"
#include "ref_data.h"

using namespace nav;
using namespace math;

static nav::Planner gPlanner;  // ~60 KB, keep it off the stack
static nav::Navigator gNav;
static nav::Path gPath;

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
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.0f, blockClearance({10, 0}, b));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, blockClearance({2, 1}, b));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, rayBlock({-10, 0}, {1, 0}, b, 120));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 120.0f, rayBlock({-10, 20}, {1, 0}, b, 120));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 120.0f, rayBlock({-10, 0}, {-1, 0}, b, 120));  // block behind the ray
    Block rot(0, 0, PI / 4);  // corner pointing at the origin axis
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 10.0f - BLOCK_HALF_DIAG, rayBlock({-10, 0}, {1, 0}, rot, 120));
    TEST_ASSERT_TRUE(insideTrack({30, -8}, 0));
    TEST_ASSERT_TRUE(insideTrack({90, 90}, 0));
    TEST_ASSERT_FALSE(insideTrack({90, 30}, 0));  // the forbidden middle of the U
    TEST_ASSERT_TRUE(insideTrack({61, 30}, TAPE_HALF + 0.5f));
}

void test_odometry_straight_and_turn() {
    FusedOdometry odo;
    odo.reset({0, 0, PI / 2});
    for (int i = 0; i < 30; i++) odo.update(56, 56, 0.0f, 1.0f / 30.0f);
    float cm = 56 * 2.0f * PI * 1.6f / 420.0f * 30;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, cm, odo.pose.y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, odo.pose.x);

    odo.reset({0, 0, 0});
    for (int i = 0; i < 30; i++) odo.update(-10, 10, PI / 2, 1.0f / 30.0f);  // 90 deg in 1 s
    TEST_ASSERT_FLOAT_WITHIN(0.02f, PI / 2, odo.pose.th);

    odo.reset({0, 0, 0});
    for (int i = 0; i < 60; i++) odo.update(0, 0, 0.01f, 1.0f / 30.0f, true);  // bias calibration
    TEST_ASSERT_TRUE(odo.bias > 0.0092f && odo.bias <= 0.01f);
}

void test_speed_controller() {
    control::Pid pid(0.004f, 0.02f, 0.0f, 20.0f, 0.3f);
    control::SpeedController c(62.2f, true, pid);
    // on target: the PID adds nothing, only the feed-forward remains
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, c.update(31.1f, 31.1f, 0.005f));
    // too slow: more duty than the feed-forward alone
    TEST_ASSERT_TRUE(c.update(31.1f, 20.0f, 0.005f) > 0.5f);
    // a target of about zero resets the PID: the wound-up integral is gone afterwards
    c.update(0.5f, 0.0f, 0.005f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, c.update(31.1f, 31.1f, 0.005f));
    // open loop: feed-forward only, whatever the measurement says
    control::SpeedController open(62.2f, false, pid);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, open.update(-31.1f, 0.0f, 0.005f));
}

void test_counter_delta_wraps() {
    TEST_ASSERT_EQUAL_INT32(5, control::counterDelta(105, 100));
    TEST_ASSERT_EQUAL_INT32(-5, control::counterDelta(95, 100));
    TEST_ASSERT_EQUAL_INT32(10, control::counterDelta(4, 65530));      // forward through the 16-bit wrap
    TEST_ASSERT_EQUAL_INT32(-10, control::counterDelta(65530, 4));     // backward through it
    TEST_ASSERT_EQUAL_INT32(0, control::counterDelta(777, 777));
}

void test_speed_estimator() {
    const float cmPerCount = 0.024f, dt = 0.005f;
    control::SpeedEstimator est(cmPerCount, dt);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.speed());
    // a steady 10 counts per period is 48 cm/s; the first-order filter (alpha 0.4) gets there from zero
    float v = 0;
    for (int i = 0; i < 40; i++) v = est.update(10);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10 * cmPerCount / dt, v);
    // the first sample alone only moves it alpha of the way
    control::SpeedEstimator fresh(cmPerCount, dt);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.4f * 10 * cmPerCount / dt, fresh.update(10));
    // backwards counts give a negative speed
    for (int i = 0; i < 40; i++) v = est.update(-10);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -10 * cmPerCount / dt, v);
    est.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.speed());
}

void test_planner_matches_python() {
    Block blocks[MAX_MAP];
    buildMap(blocks);
    gPlanner.setMap(blocks, MAP_BLOCK_COUNT);
    TEST_ASSERT_TRUE(gPlanner.plan({30, -8}, gPath));
    TEST_ASSERT_TRUE(gPath.n > 50);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * REF_PATH_LENGTH, REF_PATH_LENGTH, gPath.s[gPath.n - 1]);
    TEST_ASSERT_TRUE(gPath.p[gPath.n - 1].y < -10.0f);
    TEST_ASSERT_TRUE(gPath.p[gPath.n - 1].x > 135.0f && gPath.p[gPath.n - 1].x < 165.0f);
    for (int i = 0; i < gPath.n; i++) {
        if (gPath.p[i].y < -2.0f) continue;  // below the line is the run-through
        TEST_ASSERT_TRUE(insideTrack(gPath.p[i], -2.0f));
        TEST_ASSERT_TRUE(gPlanner.clearance(gPath.p[i]) >= 4.0f);
    }
}

// --- closed loop on the host: ideal differential-drive car, ToF from the real world -------------
namespace {

struct World {
    Block blocks[MAX_MAP + 4];
    int n = 0;
};

void tofFromWorld(const Pose& p, const World& w, float out[NUM_TOF]) {
    float c = cosf(p.th), s = sinf(p.th);
    for (int k = 0; k < NUM_TOF; k++) {
        Vec2 o = {p.x + c * tof::MOUNT_CM[k][0] - s * tof::MOUNT_CM[k][1], p.y + s * tof::MOUNT_CM[k][0] + c * tof::MOUNT_CM[k][1]};
        float best = tof::MAX_CM;
        for (int f = 0; f < tof::FOV_RAYS; f++) {
            float a = p.th + tof::ANGLE[k] + tof::FOV[f];
            Vec2 d = {cosf(a), sinf(a)};
            for (int b = 0; b < w.n; b++) best = fminf(best, rayBlock(o, d, w.blocks[b], tof::MAX_CM));
        }
        out[k] = std::clamp(best, tof::MIN_CM, tof::MAX_CM);
    }
}

bool bodyOverlaps(const Pose& p, const World& w) {
    float c = cosf(p.th), s = sinf(p.th);
    for (int ix = -2; ix <= 2; ix++) {
        for (int iy = -2; iy <= 2; iy++) {
            float lx = ix * 3.125f, ly = iy * 2.5f;  // 12.5 x 10 cm body
            Vec2 q = {p.x + c * lx - s * ly, p.y + s * lx + c * ly};
            for (int b = 0; b < w.n; b++)
                if (blockClearance(q, w.blocks[b]) < 0.3f) return true;
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
    world.blocks[world.n++] = Block(170.0f, 95.0f, 0.3f);   // random block, top-right cell
    world.blocks[world.n++] = Block(170.0f, 15.0f, -0.2f);  // random block, finish cell

    gNav = Navigator();
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    Pose truth = {30.0f, -8.0f, PI / 2};
    gNav.start(truth, 0.0f);

    const float dt = 1.0f / 30.0f;
    const float tau = 0.06f;
    float w[2] = {0, 0};  // wheel speeds, cm/s
    bool finished = false, hit = false;
    float t = 0, prevV = 0;
    for (; t < 60.0f && !finished; t += dt) {
        float tof[NUM_TOF];
        tofFromWorld(truth, world, tof);
        float duty[2];
        gNav.step(truth, prevV, tof, t, duty);  // perfect localisation: this test is about planning and tracking
        for (int k = 0; k < 3; k++) {            // physics substeps
            for (int i = 0; i < 2; i++) w[i] += (duty[i] * V_FULL * 0.925f - w[i]) * (1.0f - expf(-dt / 3 / tau));
            float v = 0.5f * (w[0] + w[1]), om = (w[1] - w[0]) / WHEEL_TRACK;
            float mid = truth.th + 0.5f * om * dt / 3;
            truth.x += v * cosf(mid) * dt / 3;
            truth.y += v * sinf(mid) * dt / 3;
            truth.th += om * dt / 3;
            prevV = v;
        }
        if (bodyOverlaps(truth, world)) hit = true;
        finished = truth.y < -9.0f && truth.x > 125.0f && truth.x < 175.0f && gNav.progress() > 0.8f;
    }
    TEST_ASSERT_TRUE_MESSAGE(finished, "did not reach the finish line");
    TEST_ASSERT_FALSE_MESSAGE(hit, "the body overlapped a block");
    TEST_ASSERT_TRUE_MESSAGE(t < 40.0f, "too slow");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_floodfill_center_and_boundaries);
    RUN_TEST(test_floodfill_wall_updates_and_known_routes);
    RUN_TEST(test_geometry);
    RUN_TEST(test_odometry_straight_and_turn);
    RUN_TEST(test_speed_controller);
    RUN_TEST(test_counter_delta_wraps);
    RUN_TEST(test_speed_estimator);
    RUN_TEST(test_planner_matches_python);
    RUN_TEST(test_closed_loop_with_unmapped_blocks);
    return UNITY_END();
}
