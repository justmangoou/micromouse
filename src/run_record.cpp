/**
 * @file    run_record.cpp
 * @brief   Run records in flash (see run_record.h). The byte layout is what tools/read_run.py reads: change both together.
 */
#include "run_record.h"

#include <string.h>

#include "board.h"
#include "config.h"
#include "motor.h"

namespace runrec {
namespace {

// ---- The trace: summary + a pose every 400 ms -------------------------------------------------------------------------

constexpr int TRACE_N = 48;  // 400 ms samples: 19 s
struct Trace {
    uint32_t samples;
    uint32_t replans;
    uint32_t unmapped;     // random blocks found
    uint32_t doneAtMs;     // ms after the start when the run ended (0 = did not end)
    uint32_t stackFree;    // bytes the stack never reached (shared with the heap)
    uint8_t state, fault;  // latest reliability::State / Fault
    uint8_t lastFault;     // last fault of the run (0 = none; FAULT_RESET_DURING_RUN = the chip reset)
    uint8_t resetFlags;    // RCC_CSR bits 31..24 at boot: 27 power-on / brown-out, 26 reset pin, 28 software, 29/30 watchdog
    uint8_t blocksFound;
    int16_t blockX[nav::MAX_EXTRA], blockY[nav::MAX_EXTRA];  // where the random blocks were found, mm
    int16_t x[TRACE_N], y[TRACE_N], thMrad[TRACE_N];
    uint8_t stateFault[TRACE_N];                           // state << 4 | fault
};
volatile Trace gTrace = {};

// ---- Flash slots ------------------------------------------------------------------------------------------------------

constexpr uint32_t SLOT_ADDR[2] = {0x0800D800, 0x0800EC00};
constexpr uint32_t SLOT_PAGES = 5;
constexpr uint32_t PAGE_BYTES = 1024;
constexpr uint32_t TRACE_BYTES = 512;  // header + trace, at the start of the slot
constexpr uint32_t RECORD_BYTES = 16;
constexpr uint32_t MAX_RECORDS = (SLOT_PAGES * PAGE_BYTES - TRACE_BYTES) / RECORD_BYTES;  // 288
constexpr uint32_t SLOT_MAGIC = 0xC0DE0002;
struct SlotHeader {
    uint32_t magic, runNumber;
};
static_assert(sizeof(SlotHeader) + sizeof(Trace) <= TRACE_BYTES, "the trace must fit the start of its slot");
static_assert(cfg::TOF_COUNT == 3, "a log record has three ToF fields");

uint32_t gSlot = 0, gRunNumber = 0;
uint32_t gRecords = 0;
uint32_t gPagesErased = 0;
bool gSlotsUsable = false;  // the program does not reach into the slots
bool gSlotReady = false;    // the slot is erased: the log may be written
bool gSaved = true;         // this run's trace is in flash (true until a run is prepared)

extern "C" uint32_t _sidata, _sdata, _edata;  // linker: end of the program image in flash

uint32_t runNumberIn(uint32_t slot) {
    const SlotHeader* h = reinterpret_cast<const SlotHeader*>(SLOT_ADDR[slot]);
    return h->magic == SLOT_MAGIC ? h->runNumber : 0;
}

// Halfwords, little endian. The area must be erased.
void flashWrite(uint32_t addr, const uint8_t* data, uint32_t bytes) {
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < bytes; i += 2) {
        const uint16_t h = static_cast<uint16_t>(data[i] | ((i + 1 < bytes ? data[i + 1] : 0xFF) << 8));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, h);
    }
    HAL_FLASH_Lock();
}

// Compact number formats of a log record.
int8_t toInt8(float v, float unit) {
    const float q = v / unit;
    return static_cast<int8_t>(q > 127.0f ? 127 : q < -128.0f ? -128 : static_cast<int>(q < 0 ? q - 0.5f : q + 0.5f));
}
uint8_t rangeTo5mm(float mm) {
    const float q = mm / 5.0f;
    return static_cast<uint8_t>(q > 255.0f ? 255 : q < 0.0f ? 0 : static_cast<int>(q + 0.5f));
}
int16_t toInt16(float v) { return static_cast<int16_t>(v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : v); }

// ---- Stack usage: the free stack is painted at boot; the deepest overwritten word is how far the stack has gone -------

extern "C" uint32_t _end;  // end of .bss: the stack grows down toward it
constexpr uint32_t STACK_PAINT = 0xA5A5A5A5u;
uint32_t* gPaintTop = nullptr;

void paintStack() {
    uint32_t* p = &_end;
    gPaintTop = reinterpret_cast<uint32_t*>(__get_MSP()) - 16;
    while (p < gPaintTop) *p++ = STACK_PAINT;
}

uint32_t stackFreeBytes() {
    uint32_t* p = gPaintTop;
    while (p > &_end && *(p - 1) != STACK_PAINT) --p;
    return static_cast<uint32_t>(reinterpret_cast<uint8_t*>(p) - reinterpret_cast<uint8_t*>(&_end));
}

// ---- "A run is in progress", in backup register DR1 (kept through resets and most brown-outs) -------------------------

constexpr uint16_t RUN_MARK = 0xA55A;
constexpr uint8_t FAULT_RESET_DURING_RUN = 8;
bool gMarked = false;

}  // namespace

bool boot() {
    paintStack();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    gTrace.resetFlags = static_cast<uint8_t>(RCC->CSR >> 24);
    RCC->CSR |= RCC_CSR_RMVF;
    if (BKP->DR1 != RUN_MARK) return false;
    gTrace.lastFault = FAULT_RESET_DURING_RUN;
    BKP->DR1 = 0;
    return true;
}

void prepare() {
    const uint32_t a = runNumberIn(0), b = runNumberIn(1);
    gSlot = a <= b ? 0 : 1;  // the older one
    gRunNumber = (a > b ? a : b) + 1;
    gRecords = 0;
    gPagesErased = 0;
    gSlotReady = false;
    gSaved = false;
    const uint32_t programEnd = reinterpret_cast<uint32_t>(&_sidata) + static_cast<uint32_t>((&_edata - &_sdata) * 4);
    gSlotsUsable = programEnd <= SLOT_ADDR[0];
}

void eraseStep() {
    if (!gSlotsUsable || gPagesErased >= SLOT_PAGES) return;
    FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_PAGES, FLASH_BANK_1, SLOT_ADDR[gSlot] + gPagesErased * PAGE_BYTES, 1};
    uint32_t pageError = 0;
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef ok = HAL_FLASHEx_Erase(&erase, &pageError);
    HAL_FLASH_Lock();
    if (ok != HAL_OK) {
        gSlotsUsable = false;
        return;
    }
    if (++gPagesErased == SLOT_PAGES) gSlotReady = true;
}

void startRun() {
    for (int i = 0; i < TRACE_N; ++i) {
        gTrace.x[i] = gTrace.y[i] = gTrace.thMrad[i] = 0;
        gTrace.stateFault[i] = 0;
    }
    gTrace.samples = 0;
    gTrace.doneAtMs = 0;
    gTrace.lastFault = 0;
}

void markRunning(bool running) {
    if (running == gMarked) return;
    BKP->DR1 = running ? RUN_MARK : 0;
    gMarked = running;
}

void logStep(uint32_t msSinceStart, const math::Pose& pose, float alpha, const float targetMmS[2], const float tofMm[]) {
    if (!gSlotReady || gRecords >= MAX_RECORDS) return;
    float measuredL, measuredR;
    motorSpeeds(&measuredL, &measuredR);
    const int16_t x = toInt16(pose.x), y = toInt16(pose.y), th = toInt16(pose.th * 1000.0f);
    const uint16_t t = static_cast<uint16_t>(msSinceStart & 0x7FFF);  // bit 15 clear: a record never starts with 0xFFFF
    const uint8_t record[RECORD_BYTES] = {
        static_cast<uint8_t>(t), static_cast<uint8_t>(t >> 8),
        static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8),
        static_cast<uint8_t>(y), static_cast<uint8_t>(y >> 8),
        static_cast<uint8_t>(th), static_cast<uint8_t>(th >> 8),
        static_cast<uint8_t>(toInt8(alpha, 0.01f)),                // steering angle, 10 mrad
        static_cast<uint8_t>(toInt8(targetMmS[0], 5.0f)),          // wheel targets, 5 mm/s
        static_cast<uint8_t>(toInt8(targetMmS[1], 5.0f)),
        static_cast<uint8_t>(toInt8(measuredL, 5.0f)),             // measured wheel speeds, 5 mm/s
        static_cast<uint8_t>(toInt8(measuredR, 5.0f)),
        rangeTo5mm(tofMm[0]), rangeTo5mm(tofMm[1]), rangeTo5mm(tofMm[2]),  // ToF right-45, front, left-45, 5 mm
    };
    flashWrite(SLOT_ADDR[gSlot] + TRACE_BYTES + gRecords * RECORD_BYTES, record, RECORD_BYTES);
    ++gRecords;
}

void traceStep(const reliability::Supervisor& supervisor, const nav::Navigator& navigator, const math::Pose& pose,
               uint32_t now) {
    using reliability::State;
    static uint32_t lastSampleAt = 0, lastStackCheck = 0;
    if (now - lastStackCheck >= 1000) {
        lastStackCheck = now;
        gTrace.stackFree = stackFreeBytes();
    }
    gTrace.state = static_cast<uint8_t>(supervisor.state());
    gTrace.fault = static_cast<uint8_t>(supervisor.fault());
    if (supervisor.started() && supervisor.fault() != reliability::Fault::None) gTrace.lastFault = gTrace.fault;
    gTrace.replans = navigator.replans();
    gTrace.unmapped = navigator.unmappedBlocks();
    gTrace.blocksFound = static_cast<uint8_t>(navigator.unmappedBlocks());
    for (int i = 0; i < navigator.unmappedBlocks() && i < nav::MAX_EXTRA; ++i) {
        gTrace.blockX[i] = static_cast<int16_t>(navigator.planner().extra(i).x);
        gTrace.blockY[i] = static_cast<int16_t>(navigator.planner().extra(i).y);
    }
    if (!supervisor.started()) return;

    // A pose sample every 400 ms, and one more when the run ends.
    const bool ended = supervisor.state() == State::Done || supervisor.state() == State::Fault;
    if (ended && gTrace.doneAtMs == 0) gTrace.doneAtMs = now - supervisor.startedAt();
    const uint32_t n = gTrace.samples;
    if (n >= TRACE_N || (!ended && now - lastSampleAt < 400 && n > 0)) return;
    if (ended && gTrace.doneAtMs != now - supervisor.startedAt() && n > 0) return;
    lastSampleAt = now;
    gTrace.x[n] = static_cast<int16_t>(pose.x);
    gTrace.y[n] = static_cast<int16_t>(pose.y);
    gTrace.thMrad[n] = static_cast<int16_t>(pose.th * 1000.0f);
    gTrace.stateFault[n] = static_cast<uint8_t>((gTrace.state << 4) | gTrace.fault);
    gTrace.samples = n + 1;
}

bool saveIfEnded(const reliability::Supervisor& supervisor) {
    using reliability::State;
    const bool ended = supervisor.state() == State::Done || supervisor.state() == State::Fault;
    if (gSaved || !supervisor.started() || !ended) return false;
    gSaved = true;  // once per run: a Recovering stop is not the end, the run resumes
    if (!gSlotsUsable || !gSlotReady) return true;
    uint8_t buffer[sizeof(SlotHeader) + sizeof(Trace)];
    const SlotHeader header = {SLOT_MAGIC, gRunNumber};
    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(header), const_cast<const Trace*>(&gTrace), sizeof(Trace));
    flashWrite(SLOT_ADDR[gSlot], buffer, sizeof(buffer));
    return true;
}

}  // namespace runrec
