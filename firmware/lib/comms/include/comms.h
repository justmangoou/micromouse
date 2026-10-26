/**
 * @file    comms.h
 * @brief   Packet protocol over a byte stream (HC-05 or a wired UART): framing, CRC, and the robot's messages.
 *          Mirrored by tools/console.py; change both together.
 *
 * @par Frame
 *   COBS(type, seq, payload..., CRC-16/CCITT-FALSE little-endian) followed by 0x00. COBS removes every 0x00 from
 *   the body, so 0x00 always marks a frame boundary and a receiver resyncs at the next one after any error.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace comms {

constexpr size_t MAX_PAYLOAD = 64;
constexpr size_t MAX_BODY = 2 + MAX_PAYLOAD + 2;               // type, seq, payload, CRC
constexpr size_t MAX_FRAME = MAX_BODY + MAX_BODY / 254 + 2;  // COBS overhead and the 0x00 delimiter

enum class Type : uint8_t {
    // robot -> host
    Telemetry = 0x01,
    Log = 0x02,  // text, normally printf output
    Ack = 0x03,  // AckPayload: every host command is answered with one
    // host -> robot, no payload
    Start = 0x10,  // same as pressing the button
    Stop = 0x11,   // end the run, motors off
    Ping = 0x12,
    Drive = 0x13,  // DrivePayload: wheel speeds for manual mode; resend faster than the robot's drive timeout
};

struct Packet {
    Type type = Type::Log;
    uint8_t seq = 0;
    uint8_t length = 0;
    uint8_t payload[MAX_PAYLOAD] = {};
};

uint16_t crc16(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF);

// Writes one complete frame into out. Returns its length, or 0 if the payload or the frame does not fit.
size_t encode(Type type, uint8_t seq, const uint8_t* payload, size_t length, uint8_t* out, size_t capacity);

// Streaming receiver: feed bytes as they arrive.
class Decoder {
public:
    // True when byte completed a valid frame; it is then in packet() until the next push().
    bool push(uint8_t byte);
    const Packet& packet() const { return packet_; }
    uint32_t errors() const { return errors_; }  // frames dropped: too long, bad COBS, too short, bad CRC

private:
    uint8_t buffer_[MAX_FRAME];
    size_t length_ = 0;
    bool overflow_ = false;
    Packet packet_;
    uint32_t errors_ = 0;
};

constexpr int TELEMETRY_RANGES = 4;

// Engineering units here; pack() quantizes (mm, mrad, mm/s as int16, ranges as uint16 mm) and saturates.
struct Telemetry {
    uint32_t ms = 0;  // HAL tick
    float x = 0, y = 0, th = 0;  // mm, mm, rad
    float v = 0;                 // forward speed, mm/s
    float target[2] = {};        // left, right wheel targets, mm/s
    float range[TELEMETRY_RANGES] = {};  // mm
    uint8_t validRanges = 0;     // bit i: range i is fresh valid data
    uint8_t state = 0;           // reliability::State
    uint8_t fault = 0;           // reliability::Fault
};
constexpr size_t TELEMETRY_SIZE = 4 + 6 * 2 + TELEMETRY_RANGES * 2 + 3;

size_t pack(const Telemetry& t, uint8_t* out);  // writes TELEMETRY_SIZE bytes
bool unpack(const Packet& p, Telemetry& t);

// Wheel speed targets in mm/s, forward positive; quantized to int16 and saturated like telemetry.
constexpr size_t DRIVE_SIZE = 4;
size_t packDrive(float left, float right, uint8_t* out);  // writes DRIVE_SIZE bytes
bool unpackDrive(const Packet& p, float& left, float& right);

struct AckPayload {
    Type type;    // the command's type
    uint8_t seq;  // the command's seq
};
constexpr size_t ACK_SIZE = 2;

}  // namespace comms
