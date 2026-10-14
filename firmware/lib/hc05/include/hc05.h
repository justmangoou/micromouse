#pragma once

#include <optional>
#include "stm32f4xx_hal.h"

// HC-05 transparent UART transport. Blocking/polled, main-loop use only.
// The caller owns GPIO/clocks, module power and KEY/EN (AT-mode selection).
class Hc05 {
public:
    explicit Hc05(UART_HandleTypeDef* uart, uint32_t timeoutMs = 100)
        : uart_(uart), timeoutMs_(timeoutMs) {}

    // Configure host UART only, 8N1. Does not change the module's baud rate.
    bool init(uint32_t baud);
    bool write(const uint8_t* data, uint16_t length);
    bool write(const char* text);
    std::optional<uint8_t> readByte(uint32_t timeoutMs = 0);
    // Send one AT command without a terminator; appends CRLF. Module must
    // already be in AT mode. Success means sent, not accepted by the module.
    bool sendAt(const char* command);
    HAL_StatusTypeDef lastStatus() const { return status_; }
    uint32_t lastError() const { return uart_ ? uart_->ErrorCode : 0; }

private:
    UART_HandleTypeDef* uart_;
    uint32_t timeoutMs_;
    HAL_StatusTypeDef status_ = HAL_OK;
};
