/**
 * Host unit tests for every math helper. Run: pio test -e native -f test_math
 */
#include <unity.h>
#include <cmath>
#include <limits>
#include "math_util.h"

void setUp() {}
void tearDown() {}

void test_toRadian() {
    static_assert(math::toRadian(0) == 0);
    static_assert(math::toRadian(90) == math::PI / 2);
    const float degrees[] = {0, 45, -90, 180, -360, 720};
    const float expected[] = {0, 0.78539816f, -1.57079633f, 3.14159265f, -6.28318531f, 12.56637061f};
    for (unsigned i = 0; i < 6; ++i)
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected[i], math::toRadian(degrees[i]));
}

void test_toRadians() {
    constexpr float degrees[] = {0, 90, -180, 360};
    constexpr auto radians = math::toRadians(degrees);
    static_assert(radians.size() == 4);
    static_assert(radians[0] == 0 && radians[1] == math::PI / 2);
    static_assert(radians[2] == -math::PI && radians[3] == 2 * math::PI);
    const float expected[] = {0, 1.57079633f, -3.14159265f, 6.28318531f};
    for (unsigned i = 0; i < 4; ++i)
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected[i], radians[i]);
    const float single[] = {-45};
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.78539816f, math::toRadians(single)[0]);
}

void test_distance() {
    const math::Vec2 a{1, -2}, b{4, 2};
    TEST_ASSERT_EQUAL_FLOAT(5, math::distance(a, b));
    TEST_ASSERT_EQUAL_FLOAT(5, math::distance(b, a));
    TEST_ASSERT_EQUAL_FLOAT(0, math::distance(a, a));
    TEST_ASSERT_EQUAL_FLOAT(7, math::distance({-3, 1}, {4, 1}));
    TEST_ASSERT_EQUAL_FLOAT(8, math::distance({2, -4}, {2, 4}));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.41421356f, math::distance({0, 0}, {1, 1}));
}

void test_distance_large_coordinates() {
    // hypotf should avoid overflow from squaring these representable coordinates.
    const float result = math::distance({0, 0}, {3e20f, 4e20f});
    TEST_ASSERT_TRUE(std::isfinite(result));
    TEST_ASSERT_FLOAT_WITHIN(1e14f, 5e20f, result);
}

void test_lerp() {
    constexpr math::Vec2 a{1, -2}, b{4, 2};
    constexpr auto mid = math::lerp(a, b, 0.5f);
    static_assert(mid.x == 2.5f && mid.y == 0);
    for (float t : {0.0f, 1.0f}) {
        const auto result = math::lerp(a, b, t);
        const auto expected = t == 0 ? a : b;
        TEST_ASSERT_EQUAL_FLOAT(expected.x, result.x);
        TEST_ASSERT_EQUAL_FLOAT(expected.y, result.y);
    }
    const auto same = math::lerp(a, a, 0.3f);
    TEST_ASSERT_EQUAL_FLOAT(a.x, same.x);
    TEST_ASSERT_EQUAL_FLOAT(a.y, same.y);
}

void test_lerp_extrapolates() {
    const auto before = math::lerp({1, -2}, {4, 2}, -1);
    TEST_ASSERT_EQUAL_FLOAT(-2, before.x);
    TEST_ASSERT_EQUAL_FLOAT(-6, before.y);
    const auto after = math::lerp({1, -2}, {4, 2}, 2);
    TEST_ASSERT_EQUAL_FLOAT(7, after.x);
    TEST_ASSERT_EQUAL_FLOAT(6, after.y);
}

void test_wrap_boundaries() {
    TEST_ASSERT_EQUAL_FLOAT(0, math::wrap(0));
    TEST_ASSERT_EQUAL_FLOAT(math::PI, math::wrap(math::PI));
    TEST_ASSERT_EQUAL_FLOAT(-math::PI, math::wrap(-math::PI));
    TEST_ASSERT_EQUAL_FLOAT(0, math::wrap(2 * math::PI));
    TEST_ASSERT_EQUAL_FLOAT(0, math::wrap(-2 * math::PI));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -math::PI + 0.01f, math::wrap(math::PI + 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, math::PI - 0.01f, math::wrap(-math::PI - 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(0.3f, math::wrap(0.3f));
}

void test_wrap_multiple_revolutions_and_large_angles() {
    for (float sign : {-1.0f, 1.0f}) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, sign * 0.3f, math::wrap(sign * (8 * math::PI + 0.3f)));
        const float large = math::wrap(sign * 1e30f);
        TEST_ASSERT_TRUE(large >= -math::PI && large <= math::PI);
        TEST_ASSERT_EQUAL_FLOAT(large, math::wrap(large));
    }
}

void test_wrap_nonfinite() {
    const float inf = std::numeric_limits<float>::infinity();
    TEST_ASSERT_TRUE(std::isnan(math::wrap(inf)));
    TEST_ASSERT_TRUE(std::isnan(math::wrap(-inf)));
    TEST_ASSERT_TRUE(std::isnan(math::wrap(std::numeric_limits<float>::quiet_NaN())));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_toRadian);
    RUN_TEST(test_toRadians);
    RUN_TEST(test_distance);
    RUN_TEST(test_distance_large_coordinates);
    RUN_TEST(test_lerp);
    RUN_TEST(test_lerp_extrapolates);
    RUN_TEST(test_wrap_boundaries);
    RUN_TEST(test_wrap_multiple_revolutions_and_large_angles);
    RUN_TEST(test_wrap_nonfinite);
    return UNITY_END();
}
