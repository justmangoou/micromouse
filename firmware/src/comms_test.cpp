/**
 * @file    comms_test.cpp
 * @brief   Stationary link check (env:comms_test): run tools/console.py on the host. Sends a Log line and a telemetry
 *          packet with uptime once a second and acknowledges commands; Start lights the status LED, Stop clears it.
 *          The wheel control interrupt is never started. Inspect gCommsTest and gRemote through ST-Link.
 */
#include <stdio.h>

#include "board.h"
#include "remote.h"

#ifndef COMMS_TEST_BAUD
#define COMMS_TEST_BAUD 9600
#endif

struct CommsTestStatus {
    uint32_t baud;
    uint32_t commands;
    uint32_t lastCommand;
};
volatile CommsTestStatus gCommsTest = {};

int main() {
    boardInit();  // Zero motor PWM.
    huart1.Init.BaudRate = COMMS_TEST_BAUD;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (true) {
            ledStatusSet((HAL_GetTick() / 100) % 2 == 0);
        }
    }
    gCommsTest.baud = COMMS_TEST_BAUD;

    uint32_t sequence = 0;
    uint32_t lastMessage = HAL_GetTick();
    while (true) {
        uint32_t now = HAL_GetTick();
        if (now - lastMessage >= 1000) {
            lastMessage = now;
            printf("comms test seq=%lu baud=%lu\n", (unsigned long)sequence++, (unsigned long)COMMS_TEST_BAUD);
            comms::Telemetry t;
            t.ms = now;
            uint8_t payload[comms::TELEMETRY_SIZE];
            remoteSend(comms::Type::Telemetry, payload, comms::pack(t, payload));
        }
        comms::Packet p;
        while (remoteReceive(p)) {
            if (p.type == comms::Type::Start) ledStatusSet(true);
            else if (p.type == comms::Type::Stop) ledStatusSet(false);
            else if (p.type != comms::Type::Ping) continue;
            ++gCommsTest.commands;
            gCommsTest.lastCommand = static_cast<uint32_t>(p.type);
            const uint8_t ack[comms::ACK_SIZE] = {static_cast<uint8_t>(p.type), p.seq};
            remoteSend(comms::Type::Ack, ack, sizeof(ack));
        }
    }
}
