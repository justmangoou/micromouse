#include <unity.h>
#include "../robot_fixture.h"
#include <limits>
#include "reliability.h"
#include "navigator.h"
#include "micromouse.h"
#include "odometry.h"
#include "../../include/map_data.h"
using namespace reliability;
static nav::Navigator navigator(cfg::TOF_LAYOUT);
void setUp() {}
void tearDown() {}

void test_freshness_and_lease_wrap() {
    uint32_t start = UINT32_MAX - 40;
    TEST_ASSERT_TRUE(fresh(30, start));
    TEST_ASSERT_FALSE(fresh(200, start));
    CommandLease lease;
    TEST_ASSERT_FALSE(lease.expired(1000));
    lease.publish(start);
    TEST_ASSERT_FALSE(lease.expired(start + 149));
    TEST_ASSERT_TRUE(lease.expired(start + 150));
    lease.clear();
    TEST_ASSERT_FALSE(lease.expired(start + 151));
}

void test_recovery_requires_continuous_health_and_stationarity() {
    Supervisor s;
    s.update(0, Fault::None, true, false);
    s.update(499, Fault::None, true, false);
    TEST_ASSERT_EQUAL((int)State::Recovering, (int)s.state());
    s.update(500, Fault::None, true, true);
    TEST_ASSERT_EQUAL((int)State::Running, (int)s.state());
    TEST_ASSERT_EQUAL_UINT32(500, s.startedAt());
    s.update(510, Fault::Sensor, false, false);
    s.update(520, Fault::None, true, false);
    s.update(1000, Fault::None, false, false);
    s.update(1010, Fault::None, true, false);
    s.update(1509, Fault::None, true, false);
    TEST_ASSERT_EQUAL((int)State::Recovering, (int)s.state());
    s.update(1510, Fault::None, true, false);
    TEST_ASSERT_EQUAL((int)State::Running, (int)s.state());
    TEST_ASSERT_EQUAL_UINT32(500, s.startedAt());
    s.latch(Fault::Pose);
    s.update(5000, Fault::None, true, true);
    TEST_ASSERT_EQUAL((int)State::Fault, (int)s.state());
}

void test_no_path_never_commands_motion() {
    float blocked[nav::MAX_MAP][3];
    // Seal the entire left corridor with a thick row of overlapping blocks.
    int n = 0;
    for (int y : {100, 200, 300})
        for (int x = 0; x <= 600; x += 100) {
            blocked[n][0] = (float)x; blocked[n][1] = (float)y; blocked[n++][2] = 0;
        }
    navigator.init(blocked, n);
    TEST_ASSERT_FALSE(navigator.start({300, -80, math::PI / 2}, 0));
    TEST_ASSERT_FALSE(navigator.hasRoute());
    float ranges[4] = {500, 500, 500, 500}, command[2] = {1, 1};
    navigator.step({300, -80, math::PI / 2}, 0, ranges, 1, command);
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
    TEST_ASSERT_EQUAL_FLOAT(0, command[1]);
}

void test_failed_replan_clears_old_route_and_preserves_obstacles() {
    navigator.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    TEST_ASSERT_TRUE(navigator.start({300, -80, math::PI / 2}, 0));
    // A continuous barrier across the left corridor, retained across retries.
    auto& planner = const_cast<nav::Planner&>(navigator.planner());
    for (int x = 0; x <= 600; x += 100) planner.addExtra({(float)x, 100});
    TEST_ASSERT_FALSE(navigator.reposition({300, -80, math::PI / 2}, 1));
    TEST_ASSERT_EQUAL_INT(7, navigator.unmappedBlocks());
    TEST_ASSERT_EQUAL_INT(0, navigator.path().n);
    TEST_ASSERT_FALSE(navigator.reposition({300, -80, math::PI / 2}, 2));
    TEST_ASSERT_EQUAL_INT(7, navigator.unmappedBlocks());
}

void test_encoder_only_odometry() {
    nav::FusedOdometry odo(cfg::ODOM);
    odo.updateEncoders(10, 10, 0.1f);
    TEST_ASSERT_TRUE(odo.pose.x > 0);
    TEST_ASSERT_EQUAL_FLOAT(0, odo.pose.th);
    float x = odo.pose.x;
    odo.updateEncoders(-5, 5, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, x, odo.pose.x);
    TEST_ASSERT_TRUE(odo.pose.th > 0);
    float heading = odo.pose.th;
    odo.updateEncoders(1, 1, 0);
    TEST_ASSERT_EQUAL_FLOAT(heading, odo.pose.th);
}

void test_maze_pause_retains_phase_and_knowledge() {
    nav::Micromouse mouse(cfg::TOF_LAYOUT, robotMazeParams());
    TEST_ASSERT_TRUE(mouse.start(0));
    float ranges[4] = {300, 300, 300, 300};
    bool valid[4] = {true, true, true, true};
    float command[2];
    mouse.step(mouse.startPose(), ranges, valid, 0.1f, command);
    TEST_ASSERT_EQUAL((int)nav::Micromouse::State::Advancing, (int)mouse.state());
    auto passage = mouse.maze().passage({0, 0}, nav::Direction::North);
    mouse.pause(0.2f);
    mouse.step(mouse.startPose(), ranges, valid, 10, command);
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
    TEST_ASSERT_TRUE(mouse.resume(10));
    mouse.step(mouse.startPose(), ranges, valid, 10.01f, command);
    TEST_ASSERT_EQUAL((int)nav::Micromouse::State::Advancing, (int)mouse.state());
    TEST_ASSERT_TRUE(command[0] > 0);
    TEST_ASSERT_EQUAL((int)passage, (int)mouse.maze().passage({0, 0}, nav::Direction::North));
    mouse.pause(10.02f);
    TEST_ASSERT_FALSE(mouse.resume(241));
}

void test_snapshot_rejects_stale_missing_and_invalid_data() {
    SensorSnapshot<cfg::TOF_COUNT> s;
    s.now = s.gyroAt = s.completedAt = 100;
    s.dt = 0.033f;
    s.gyroSamples = 6;
    for (auto& range : s.ranges) range = {300, 100, RangeStatus::Valid, 0};
    TEST_ASSERT_EQUAL((int)Fault::None, (int)s.issue(100, 15, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(100, 15, false, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(100, 15, true, false, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.ranges[0].status = RangeStatus::Invalid;
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(100, 15, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    TEST_ASSERT_EQUAL((int)Fault::None, (int)s.issue(100, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.gyroSamples = 0;
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(100, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.gyroSamples = 1;
    s.gyro = std::numeric_limits<float>::quiet_NaN();
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(100, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.gyro = 0;
    s.dt = 0;
    TEST_ASSERT_EQUAL((int)Fault::Timing, (int)s.issue(100, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.dt = 0.151f;
    TEST_ASSERT_EQUAL((int)Fault::Timing, (int)s.issue(100, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.dt = 0.033f;
    s.completedAt = 251;
    TEST_ASSERT_EQUAL((int)Fault::Timing, (int)s.issue(251, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.now = s.completedAt = s.gyroAt = 251;
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(251, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    for (auto& range : s.ranges) range.timestamp = 251;
    TEST_ASSERT_EQUAL((int)Fault::None, (int)s.issue(251, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
    s.ranges[1].mm = std::numeric_limits<float>::infinity();
    TEST_ASSERT_EQUAL((int)Fault::Sensor, (int)s.issue(251, 6, true, true, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM));
}

void test_encoder_gap_limit_and_recovery_wrap() {
    TEST_ASSERT_TRUE(encoderGapSafe(12, 0.24f, 622));
    TEST_ASSERT_FALSE(encoderGapSafe(13, 0.24f, 622));
    TEST_ASSERT_FALSE(encoderGapSafe(32768.0f * 0.24f / 622, 0.24f, 622));
    TEST_ASSERT_FALSE(encoderGapSafe(-1, 0.24f, 622));
    Qualification q;
    uint32_t start = UINT32_MAX - 100;
    TEST_ASSERT_FALSE(q.update(start, true, true));
    TEST_ASSERT_FALSE(q.update(start + 499, true, true));
    TEST_ASSERT_TRUE(q.update(start + 500, true, true));
    TEST_ASSERT_FALSE(q.update(start + 501, false, true));
    TEST_ASSERT_FALSE(q.update(start + 502, true, true));
}

void test_maze_resume_aligns_before_advancing() {
    nav::Micromouse mouse(cfg::TOF_LAYOUT, robotMazeParams());
    TEST_ASSERT_TRUE(mouse.start(0));
    float ranges[4] = {300, 300, 300, 300};
    bool valid[4] = {true,true,true,true};
    float command[2];
    auto pose = mouse.startPose();
    mouse.step(pose, ranges, valid, 0.1f, command);
    mouse.pause(0.2f);
    TEST_ASSERT_TRUE(mouse.resume(1));
    pose.th += 0.2f;
    mouse.step(pose, ranges, valid, 1.01f, command);
    TEST_ASSERT_TRUE(command[0] * command[1] < 0);
    TEST_ASSERT_EQUAL((int)nav::Micromouse::State::Advancing, (int)mouse.state());
    pose.th = mouse.startPose().th;
    mouse.step(pose, ranges, valid, 1.02f, command);
    TEST_ASSERT_TRUE(command[0] > 0 && command[1] > 0);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_freshness_and_lease_wrap);
    RUN_TEST(test_recovery_requires_continuous_health_and_stationarity);
    RUN_TEST(test_no_path_never_commands_motion);
    RUN_TEST(test_failed_replan_clears_old_route_and_preserves_obstacles);
    RUN_TEST(test_encoder_only_odometry);
    RUN_TEST(test_maze_pause_retains_phase_and_knowledge);
    RUN_TEST(test_snapshot_rejects_stale_missing_and_invalid_data);
    RUN_TEST(test_encoder_gap_limit_and_recovery_wrap);
    RUN_TEST(test_maze_resume_aligns_before_advancing);
    return UNITY_END();
}
