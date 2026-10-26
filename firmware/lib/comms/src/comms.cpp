/**
 * @file    comms.cpp
 * @brief   COBS framing, CRC-16/CCITT-FALSE, and little-endian message layout.
 */
#include "comms.h"

#include <math.h>

namespace comms {

namespace {

size_t cobsEncode(const uint8_t* in, size_t length, uint8_t* out) {
    size_t codeAt = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < length; ++i) {
        if (in[i] == 0) {
            out[codeAt] = code;
            codeAt = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[codeAt] = code;
                codeAt = o++;
                code = 1;
            }
        }
    }
    out[codeAt] = code;
    return o;
}

// In place: the decoded data never runs ahead of the encoded data. Returns the decoded length, 0 on malformed input.
size_t cobsDecode(uint8_t* data, size_t length) {
    size_t i = 0, o = 0;
    while (i < length) {
        uint8_t code = data[i++];
        if (code == 0 || i + code - 1 > length) return 0;
        for (uint8_t k = 1; k < code; ++k) data[o++] = data[i++];
        if (code < 0xFF && i < length) data[o++] = 0;
    }
    return o;
}

int16_t saturate16(float v) {
    if (!isfinite(v)) return 0;
    if (v >= 32767.0f) return 32767;
    if (v <= -32768.0f) return -32768;
    return static_cast<int16_t>(lrintf(v));
}

uint16_t saturateU16(float v) {
    if (!isfinite(v) || v <= 0) return 0;
    if (v >= 65535.0f) return 65535;
    return static_cast<uint16_t>(lrintf(v));
}

uint8_t* put16(uint8_t* p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = v >> 8;
    return p + 2;
}

uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }

}  // namespace

uint16_t crc16(const uint8_t* data, size_t length, uint16_t crc) {
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

size_t encode(Type type, uint8_t seq, const uint8_t* payload, size_t length, uint8_t* out, size_t capacity) {
    if (length > MAX_PAYLOAD || (!payload && length)) return 0;
    uint8_t body[MAX_BODY];
    body[0] = static_cast<uint8_t>(type);
    body[1] = seq;
    for (size_t i = 0; i < length; ++i) body[2 + i] = payload[i];
    put16(body + 2 + length, crc16(body, 2 + length));
    size_t bodyLength = 4 + length;
    if (!out || capacity < bodyLength + bodyLength / 254 + 2) return 0;
    size_t n = cobsEncode(body, bodyLength, out);
    out[n++] = 0;
    return n;
}

bool Decoder::push(uint8_t byte) {
    if (byte != 0) {
        if (length_ < MAX_FRAME) buffer_[length_++] = byte;
        else overflow_ = true;
        return false;
    }
    size_t length = length_;
    bool overflow = overflow_;
    length_ = 0;
    overflow_ = false;
    if (length == 0) return false;  // back-to-back delimiters: idle, not an error
    size_t n = overflow ? 0 : cobsDecode(buffer_, length);
    if (n < 4 || n - 4 > MAX_PAYLOAD || crc16(buffer_, n - 2) != get16(buffer_ + n - 2)) {
        ++errors_;
        return false;
    }
    packet_.type = static_cast<Type>(buffer_[0]);
    packet_.seq = buffer_[1];
    packet_.length = static_cast<uint8_t>(n - 4);
    for (size_t i = 0; i < packet_.length; ++i) packet_.payload[i] = buffer_[2 + i];
    return true;
}

size_t pack(const Telemetry& t, uint8_t* out) {
    uint8_t* p = out;
    p = put16(p, t.ms & 0xFFFF);
    p = put16(p, t.ms >> 16);
    p = put16(p, saturate16(t.x));
    p = put16(p, saturate16(t.y));
    p = put16(p, saturate16(t.th * 1000.0f));
    p = put16(p, saturate16(t.v));
    p = put16(p, saturate16(t.target[0]));
    p = put16(p, saturate16(t.target[1]));
    for (float r : t.range) p = put16(p, saturateU16(r));
    *p++ = t.validRanges;
    *p++ = t.state;
    *p++ = t.fault;
    return p - out;
}

bool unpack(const Packet& packet, Telemetry& t) {
    if (packet.type != Type::Telemetry || packet.length != TELEMETRY_SIZE) return false;
    const uint8_t* p = packet.payload;
    auto s16 = [&p] { int16_t v = static_cast<int16_t>(get16(p)); p += 2; return static_cast<float>(v); };
    t.ms = get16(p) | static_cast<uint32_t>(get16(p + 2)) << 16;
    p += 4;
    t.x = s16();
    t.y = s16();
    t.th = s16() * 0.001f;
    t.v = s16();
    t.target[0] = s16();
    t.target[1] = s16();
    for (float& r : t.range) {
        r = get16(p);
        p += 2;
    }
    t.validRanges = *p++;
    t.state = *p++;
    t.fault = *p++;
    return true;
}

size_t packDrive(float left, float right, uint8_t* out) {
    uint8_t* p = put16(out, saturate16(left));
    p = put16(p, saturate16(right));
    return p - out;
}

bool unpackDrive(const Packet& packet, float& left, float& right) {
    if (packet.type != Type::Drive || packet.length != DRIVE_SIZE) return false;
    left = static_cast<int16_t>(get16(packet.payload));
    right = static_cast<int16_t>(get16(packet.payload + 2));
    return true;
}

}  // namespace comms
