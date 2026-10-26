#include <unity.h>
#include <limits>
#include "tof_layout.h"
#include "micromouse.h"
#include "navigator.h"
#include "reliability.h"

static tof::Layout layout() {
    tof::Layout value;
    value.count = 3;
    value.sensors[0] = {10, -10, 0};
    value.sensors[1] = {0, 0, math::PI};  // unused rear sensor
    value.sensors[2] = {20, 10, 0};
    value.rayCount = 1;
    value.rays[0] = 0;
    value.minMm = 5;
    value.maxMm = 500;
    return value;
}

void setUp() {}
void tearDown() {}

void test_layout_validation() {
    auto value = layout();
    TEST_ASSERT_TRUE(value.valid());
    value.count = tof::MAX_SENSORS + 1;
    TEST_ASSERT_FALSE(value.valid());
    static nav::Navigator invalid(value);
    TEST_ASSERT_FALSE(invalid.configurationValid());
    TEST_ASSERT_FALSE(invalid.start({300, -80, math::PI/2}, 0));
    value = layout(); value.count = 0;
    TEST_ASSERT_FALSE(value.valid());
    value = layout(); value.rayCount = 0;
    TEST_ASSERT_FALSE(value.valid());
    value = layout(); value.rayCount = tof::MAX_FOV_RAYS + 1;
    TEST_ASSERT_FALSE(value.valid());
    value = layout(); value.maxMm = value.minMm;
    TEST_ASSERT_FALSE(value.valid());
    value = layout(); value.sensors[2].angle = std::numeric_limits<float>::quiet_NaN();
    TEST_ASSERT_FALSE(value.valid());
}

void test_maze_uses_configured_sensor_indices_mounts_and_limits() {
    nav::MicromouseParams params;
    params.leftSensor = 2;
    params.rightSensor = 0;
    nav::Micromouse mouse(layout(), params);
    TEST_ASSERT_TRUE(mouse.start(0));
    // Only 0 and 2 are forward sensors; the middle slot is intentionally unusable.
    float ranges[3] = {80, std::numeric_limits<float>::quiet_NaN(), 70};
    bool reliable[3] = {true, false, true};
    float command[2];
    mouse.step(mouse.startPose(), ranges, reliable, 0.01f, command);
    // Forward projections are both 90 mm, the cell boundary.
    TEST_ASSERT_EQUAL_INT((int)nav::Passage::Wall, (int)mouse.maze().passage({0,0}, nav::Direction::North));
    TEST_ASSERT_TRUE(mouse.start(0));
    ranges[0] = ranges[2] = 200;
    mouse.step(mouse.startPose(), ranges, reliable, 0.01f, command);
    TEST_ASSERT_EQUAL_INT((int)nav::Micromouse::State::Advancing, (int)mouse.state());
    mouse.step(mouse.startPose(), ranges, reliable, 0.02f, command);
    TEST_ASSERT_TRUE(command[0] > 0 && command[1] > 0);
    ranges[2] = 500;  // this layout's no-target sentinel
    mouse.step(mouse.startPose(), ranges, reliable, 0.03f, command);
    TEST_ASSERT_EQUAL_INT((int)nav::Micromouse::State::Fault, (int)mouse.state());
    TEST_ASSERT_EQUAL_FLOAT(0, command[0]);
    TEST_ASSERT_EQUAL_FLOAT(0, command[1]);
}

void test_maze_rejects_invalid_sensor_selection() {
    nav::MicromouseParams params;
    params.leftSensor = params.rightSensor = 0;
    nav::Micromouse same(layout(), params);
    TEST_ASSERT_FALSE(same.start(0));
    params.leftSensor = 3;
    nav::Micromouse outside(layout(), params);
    TEST_ASSERT_FALSE(outside.start(0));
    params.leftSensor = 1; // rear-facing
    nav::Micromouse rear(layout(), params);
    TEST_ASSERT_FALSE(rear.start(0));
}

void test_course_accepts_one_sensor_and_one_ray() {
    auto value = layout();
    value.count = 1;
    static nav::Navigator car(value);
    car.init(nullptr, 0);
    TEST_ASSERT_TRUE(car.configurationValid());
    TEST_ASSERT_TRUE(car.start({300, -80, math::PI/2}, 0));
    float ranges[1] = {100}, command[2];
    car.step({300, -80, math::PI/2}, 0, ranges, 0.1f, command);
    car.step({300, -80, math::PI/2}, 0, ranges, 0.15f, command);
    TEST_ASSERT_EQUAL_INT(1, car.unmappedBlocks());
}

void test_health_snapshot_is_independent_of_robot_layout() {
    reliability::SensorSnapshot<2> s;
    s.dt = 0.01f;
    s.gyroSamples = 1;
    s.ranges[0] = {10, 0, reliability::RangeStatus::Valid, 0};
    s.ranges[1] = {300, 0, reliability::RangeStatus::Valid, 0};
    TEST_ASSERT_EQUAL_INT((int)reliability::Fault::None, (int)s.issue(0, 3, true, true, 5, 500));
    TEST_ASSERT_EQUAL_INT((int)reliability::Fault::Sensor, (int)s.issue(0, 3, true, true, 20, 500));
    TEST_ASSERT_EQUAL_INT((int)reliability::Fault::Configuration, (int)s.issue(0, 4, true, true, 5, 500));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_layout_validation);
    RUN_TEST(test_maze_uses_configured_sensor_indices_mounts_and_limits);
    RUN_TEST(test_maze_rejects_invalid_sensor_selection);
    RUN_TEST(test_course_accepts_one_sensor_and_one_ray);
    RUN_TEST(test_health_snapshot_is_independent_of_robot_layout);
    return UNITY_END();
}
