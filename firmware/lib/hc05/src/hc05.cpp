#include "hc05.h"

#include <cstring>

bool Hc05::init(uint32_t baud) {
    if (!uart_ || !uart_->Instance || baud == 0) {
        status_ = HAL_ERROR;
        return false;
    }
    uart_->Init.BaudRate = baud;
    uart_->Init.WordLength = UART_WORDLENGTH_8B;
    uart_->Init.StopBits = UART_STOPBITS_1;
    uart_->Init.Parity = UART_PARITY_NONE;
    uart_->Init.Mode = UART_MODE_TX_RX;
    uart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_->Init.OverSampling = UART_OVERSAMPLING_16;
    status_ = HAL_UART_Init(uart_);
    return status_ == HAL_OK;
}

bool Hc05::write(const uint8_t* data, uint16_t length) {
    if (!uart_ || !uart_->Instance || (!data && length)) {
        status_ = HAL_ERROR;
        return false;
    }
    if (!length) {
        status_ = HAL_OK;
        return true;
    }
    status_ = HAL_UART_Transmit(uart_, const_cast<uint8_t*>(data), length, timeoutMs_);
    return status_ == HAL_OK;
}

bool Hc05::write(const char* text) {
    if (!text) {
        status_ = HAL_ERROR;
        return false;
    }
    size_t length = std::strlen(text);
    if (length > UINT16_MAX) {
        status_ = HAL_ERROR;
        return false;
    }
    return write(reinterpret_cast<const uint8_t*>(text), static_cast<uint16_t>(length));
}

std::optional<uint8_t> Hc05::readByte(uint32_t timeoutMs) {
    if (!uart_ || !uart_->Instance) {
        status_ = HAL_ERROR;
        return std::nullopt;
    }
    uint8_t byte;
    status_ = HAL_UART_Receive(uart_, &byte, 1, timeoutMs);
    if (status_ != HAL_OK) return std::nullopt;
    return byte;
}

bool Hc05::sendAt(const char* command) {
    // One bounded transfer avoids interleaving the command and terminator.
    char frame[128];
    if (!command || std::strncmp(command, "AT", 2) != 0) {
        status_ = HAL_ERROR;
        return false;
    }
    size_t length = std::strlen(command);
    if (length > sizeof(frame) - 2 || std::strchr(command, '\r') || std::strchr(command, '\n')) {
        status_ = HAL_ERROR;
        return false;
    }
    std::memcpy(frame, command, length);
    frame[length] = '\r';
    frame[length + 1] = '\n';
    return write(reinterpret_cast<const uint8_t*>(frame), static_cast<uint16_t>(length + 2));
}
