#include <unity.h>
#include <limits>
#include "tof_layout.h"
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
    RUN_TEST(test_course_accepts_one_sensor_and_one_ray);
    RUN_TEST(test_health_snapshot_is_independent_of_robot_layout);
    return UNITY_END();
}
