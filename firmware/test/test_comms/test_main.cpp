#include <unity.h>
#include "comms.h"
using namespace comms;
void setUp() {}
void tearDown() {}

static bool feed(Decoder& d, const uint8_t* frame, size_t n) {
    bool got = false;
    for (size_t i = 0; i < n; ++i) got = d.push(frame[i]);
    return got;
}

void test_crc_check_value() {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(text, sizeof(text)));
}

// Same bytes as GOLDEN_PING in tools/console.py.
void test_golden_ping_frame() {
    uint8_t out[MAX_FRAME];
    size_t n = encode(Type::Ping, 7, nullptr, 0, out, sizeof(out));
    const uint8_t expected[] = {0x05, 0x12, 0x07, 0xF9, 0x08, 0x00};
    TEST_ASSERT_EQUAL(sizeof(expected), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, n);
}

void test_round_trip_with_zero_bytes_and_full_payload() {
    uint8_t payload[MAX_PAYLOAD];
    for (size_t i = 0; i < MAX_PAYLOAD; ++i) payload[i] = i % 3 ? i : 0;
    uint8_t out[MAX_FRAME];
    size_t n = encode(Type::Log, 200, payload, sizeof(payload), out, sizeof(out));
    TEST_ASSERT_TRUE(n > 0);
    for (size_t i = 0; i + 1 < n; ++i) TEST_ASSERT_NOT_EQUAL(0, out[i]);
    Decoder d;
    TEST_ASSERT_TRUE(feed(d, out, n));
    TEST_ASSERT_EQUAL((int)Type::Log, (int)d.packet().type);
    TEST_ASSERT_EQUAL(200, d.packet().seq);
    TEST_ASSERT_EQUAL(MAX_PAYLOAD, d.packet().length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, d.packet().payload, MAX_PAYLOAD);
}

void test_encode_rejects_oversize_and_small_buffer() {
    uint8_t payload[MAX_PAYLOAD + 1] = {};
    uint8_t out[MAX_FRAME];
    TEST_ASSERT_EQUAL(0, encode(Type::Log, 0, payload, sizeof(payload), out, sizeof(out)));
    TEST_ASSERT_EQUAL(0, encode(Type::Ping, 0, nullptr, 0, out, 5));
}

void test_decoder_drops_corrupt_frames_and_resyncs() {
    uint8_t ping[MAX_FRAME];
    size_t n = encode(Type::Ping, 1, nullptr, 0, ping, sizeof(ping));
    Decoder d;
    // Plain text and idle delimiters before a frame.
    const uint8_t noise[] = {'h', 'i', '\r', '\n', 0, 0};
    TEST_ASSERT_FALSE(feed(d, noise, sizeof(noise)));
    TEST_ASSERT_EQUAL_UINT32(1, d.errors());
    TEST_ASSERT_TRUE(feed(d, ping, n));
    // Flipped bit: CRC fails.
    uint8_t bad[MAX_FRAME];
    for (size_t i = 0; i < n; ++i) bad[i] = ping[i];
    bad[3] ^= 0x01;  // a CRC byte; never produces 0x00 here
    TEST_ASSERT_FALSE(feed(d, bad, n));
    TEST_ASSERT_EQUAL_UINT32(2, d.errors());
    // A run longer than any frame, then a good frame.
    for (size_t i = 0; i < 2 * MAX_FRAME; ++i) d.push(0x55);
    d.push(0);
    TEST_ASSERT_EQUAL_UINT32(3, d.errors());
    TEST_ASSERT_TRUE(feed(d, ping, n));
    TEST_ASSERT_EQUAL(1, d.packet().seq);
}

void test_telemetry_round_trip_and_saturation() {
    Telemetry t;
    t.ms = 0x12345678;
    t.x = 1234.4f;
    t.y = -80.6f;
    t.th = 1.5708f;
    t.v = 99999.0f;  // saturates
    t.target[0] = -120.0f;
    t.target[1] = NAN;  // sent as 0
    t.range[0] = 50.0f;
    t.range[1] = -5.0f;  // sent as 0
    t.range[2] = 70000.0f;
    t.range[3] = 1200.0f;
    t.validRanges = 0b1011;
    t.state = 1;
    t.fault = 4;
    uint8_t payload[MAX_PAYLOAD];
    TEST_ASSERT_EQUAL(TELEMETRY_SIZE, pack(t, payload));
    uint8_t out[MAX_FRAME];
    size_t n = encode(Type::Telemetry, 3, payload, TELEMETRY_SIZE, out, sizeof(out));
    Decoder d;
    TEST_ASSERT_TRUE(feed(d, out, n));
    Telemetry r;
    TEST_ASSERT_TRUE(unpack(d.packet(), r));
    TEST_ASSERT_EQUAL_HEX32(0x12345678, r.ms);
    TEST_ASSERT_EQUAL_FLOAT(1234.0f, r.x);
    TEST_ASSERT_EQUAL_FLOAT(-81.0f, r.y);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 1.571f, r.th);
    TEST_ASSERT_EQUAL_FLOAT(32767.0f, r.v);
    TEST_ASSERT_EQUAL_FLOAT(-120.0f, r.target[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.target[1]);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.range[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.range[1]);
    TEST_ASSERT_EQUAL_FLOAT(65535.0f, r.range[2]);
    TEST_ASSERT_EQUAL(0b1011, r.validRanges);
    TEST_ASSERT_EQUAL(1, r.state);
    TEST_ASSERT_EQUAL(4, r.fault);
    Packet wrong = d.packet();
    wrong.length--;
    TEST_ASSERT_FALSE(unpack(wrong, r));
}

void test_drive_round_trip_and_validation() {
    uint8_t payload[DRIVE_SIZE];
    TEST_ASSERT_EQUAL(DRIVE_SIZE, packDrive(-250.4f, 99999.0f, payload));
    uint8_t out[MAX_FRAME];
    size_t n = encode(Type::Drive, 9, payload, DRIVE_SIZE, out, sizeof(out));
    Decoder d;
    TEST_ASSERT_TRUE(feed(d, out, n));
    float l = 0, r = 0;
    TEST_ASSERT_TRUE(unpackDrive(d.packet(), l, r));
    TEST_ASSERT_EQUAL_FLOAT(-250.0f, l);
    TEST_ASSERT_EQUAL_FLOAT(32767.0f, r);
    Packet wrong = d.packet();
    wrong.length = 3;
    TEST_ASSERT_FALSE(unpackDrive(wrong, l, r));
    wrong = d.packet();
    wrong.type = Type::Ping;
    TEST_ASSERT_FALSE(unpackDrive(wrong, l, r));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_golden_ping_frame);
    RUN_TEST(test_round_trip_with_zero_bytes_and_full_payload);
    RUN_TEST(test_encode_rejects_oversize_and_small_buffer);
    RUN_TEST(test_decoder_drops_corrupt_frames_and_resyncs);
    RUN_TEST(test_telemetry_round_trip_and_saturation);
    RUN_TEST(test_drive_round_trip_and_validation);
    return UNITY_END();
}
