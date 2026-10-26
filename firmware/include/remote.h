/**
 * @file    remote.h
 * @brief   comms packets over USART1 (HC-05 on PA9/PA10, or a wired adapter). Interrupt-driven RX and TX ring buffers:
 *          nothing here blocks the control loop. printf output travels as Log packets through the same buffers.
 *
 * @par Notes
 *   - Call from the main loop only (single producer); never from an interrupt.
 *   - A frame that does not fit in the TX buffer is dropped whole, never sent in part.
 *   - Starts on first use with huart1's configuration from boardInit().
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "comms.h"

bool remoteSend(comms::Type type, const uint8_t* payload = nullptr, size_t length = 0);
// printf goes here. Splits text into Log packets, or sends it raw when built with -DREMOTE_PLAIN_TEXT
// (bring-up builds read with a plain serial terminal).
void remoteWrite(const char* text, size_t length);
// Next valid packet received from the host, if any.
bool remoteReceive(comms::Packet& packet);

struct RemoteStats {
    uint32_t sent = 0;
    uint32_t txDropped = 0;   // frames that did not fit in the TX buffer
    uint32_t received = 0;
    uint32_t rxOverruns = 0;  // bytes lost: RX buffer full or UART overrun
    uint32_t rxErrors = 0;    // frames rejected by the decoder
};
extern volatile RemoteStats gRemote;  // inspect through ST-Link
