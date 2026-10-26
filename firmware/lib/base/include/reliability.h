#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace reliability {

constexpr uint32_t MAX_AGE_MS = 150;
constexpr uint32_t QUALIFY_MS = 500;
constexpr uint32_t RETRY_MS = 1000;
inline bool fresh(uint32_t now, uint32_t at, uint32_t age = MAX_AGE_MS) { return now - at <= age; }
enum class RangeStatus { Missing, Valid, Invalid, BusError, Stale, NotReady };
struct RangeReading {
    float mm = 0;
    uint32_t timestamp = 0;
    RangeStatus status = RangeStatus::Missing;
    uint32_t failures = 0;
};
enum class State { Ready, Running, Recovering, Done, Fault };
enum class Fault { None, Sensor, Timing, CommandExpired, NoRoute, Pose, Configuration, Timeout };

template <std::size_t Count>
struct SensorSnapshot {
    static_assert(Count > 0 && Count <= 32, "sensor mask supports 1..32 ranges");
    uint32_t now = 0;          // encoder sampling time
    uint32_t completedAt = 0;  // after all sensor I/O
    uint32_t gyroAt = 0;
    float dt = 0;
    float gyro = 0;
    unsigned gyroSamples = 0;
    RangeReading ranges[Count];

    Fault issue(uint32_t at, unsigned requiredMask, bool initialized, bool calibrated, float minMm, float maxMm) const {
        if constexpr (Count < 32) {
            if (requiredMask >> Count) return Fault::Configuration;
        }
        if (!std::isfinite(minMm) || !std::isfinite(maxMm) || minMm < 0 || maxMm <= minMm) return Fault::Configuration;
        if (!std::isfinite(dt) || dt <= 0 || dt > MAX_AGE_MS * 0.001f ||
            !fresh(at, now) || !fresh(at, completedAt)) return Fault::Timing;
        if (!initialized || !calibrated || !gyroSamples || !std::isfinite(gyro) || !fresh(at, gyroAt))
            return Fault::Sensor;
        for (std::size_t i = 0; i < Count; ++i) {
            const auto& r = ranges[i];
            if ((requiredMask & (1u << i)) && (r.status != RangeStatus::Valid ||
                !fresh(at, r.timestamp) || !std::isfinite(r.mm) || r.mm < minMm || r.mm >= maxMm))
                return Fault::Sensor;
        }
        return Fault::None;
    }
};

inline bool encoderGapSafe(float dt, float mmPerCount, float fullSpeed) {
    return std::isfinite(dt) && dt >= 0 && mmPerCount > 0 && fullSpeed > 0 &&
           dt < 32768.0f * mmPerCount / fullSpeed;
}


// Call with the actual acquisition-completion time. Movement restarts qualification.
class Qualification {
public:
    bool update(uint32_t now, bool healthy, bool stationary) {
        if (!healthy || !stationary) { reset(); return false; }
        if (!active_) { since_ = now; active_ = true; }
        return now - since_ >= QUALIFY_MS;
    }
    void reset() { active_ = false; }
private:
    bool active_ = false;
    uint32_t since_ = 0;
};

class Supervisor {
public:
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    bool started() const { return started_; }
    uint32_t startedAt() const { return startedAt_; }
    void latch(Fault fault) { if (state_ == State::Done) return; state_ = State::Fault; fault_ = fault; qualification_.reset(); }
    void done() { if (state_ == State::Fault) return; state_ = State::Done; fault_ = Fault::None; }
    void update(uint32_t now, Fault issue, bool stationary, bool startRequested) {
        if (state_ == State::Fault || state_ == State::Done) return;
        if (issue != Fault::None) {
            state_ = State::Recovering;
            fault_ = issue;
            qualification_.reset();
            return;
        }
        if (state_ == State::Recovering) {
            if (!qualification_.update(now, true, stationary)) return;
            state_ = started_ ? State::Running : State::Ready;
            fault_ = Fault::None;
        }
        if (state_ == State::Ready && startRequested) {
            started_ = true;
            startedAt_ = now;
            state_ = State::Running;
        }
    }
private:
    State state_ = State::Recovering;
    Fault fault_ = Fault::Sensor;
    Qualification qualification_;
    bool started_ = false;
    uint32_t startedAt_ = 0;
};

// The motor owner publishes commands and this lease under the same IRQ lock.
class CommandLease {
public:
    void publish(uint32_t now) { at_ = now; active_ = true; }
    void clear() { active_ = false; }
    bool expired(uint32_t now) const { return active_ && now - at_ >= MAX_AGE_MS; }
private:
    uint32_t at_ = 0;
    bool active_ = false;
};
}
