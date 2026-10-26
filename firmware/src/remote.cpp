/**
 * @file    remote.cpp
 * @brief   USART1 packet transport. The interrupt moves bytes between the data register and two ring buffers; the
 *          main loop frames and parses. USART1 sits below TIM11 in priority so the speed loop is never delayed.
 */
#include "remote.h"

#include "board.h"

volatile RemoteStats gRemote;

namespace {

constexpr uint32_t RX_SIZE = 256;   // powers of two
constexpr uint32_t TX_SIZE = 1024;  // ~1 s of traffic at 9600 baud

uint8_t gRx[RX_SIZE];
uint8_t gTx[TX_SIZE];
volatile uint32_t gRxHead, gRxTail;  // head written by the interrupt, tail by the main loop
volatile uint32_t gTxHead, gTxTail;  // head written by the main loop, tail by the interrupt
uint8_t gSeq = 0;
comms::Decoder gDecoder;
bool gStarted = false;

bool start() {
    if (gStarted) return true;
    if (!huart1.Instance) return false;  // before boardInit()
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    gStarted = true;
    return true;
}

// All or nothing, so the receiver never sees half a frame.
bool enqueue(const uint8_t* data, size_t length) {
    if (!start()) return false;
    uint32_t head = gTxHead;
    if (length > TX_SIZE - 1 - ((head - gTxTail) & (TX_SIZE - 1))) return false;
    for (size_t i = 0; i < length; ++i) gTx[(head + i) & (TX_SIZE - 1)] = data[i];
    gTxHead = (head + length) & (TX_SIZE - 1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    return true;
}

}  // namespace

bool remoteSend(comms::Type type, const uint8_t* payload, size_t length) {
    uint8_t frame[comms::MAX_FRAME];
    size_t n = comms::encode(type, gSeq, payload, length, frame, sizeof(frame));
    if (n == 0 || !enqueue(frame, n)) {
        ++gRemote.txDropped;
        return false;
    }
    ++gSeq;
    ++gRemote.sent;
    return true;
}

void remoteWrite(const char* text, size_t length) {
#ifdef REMOTE_PLAIN_TEXT
    if (!enqueue(reinterpret_cast<const uint8_t*>(text), length)) ++gRemote.txDropped;
#else
    while (length > 0) {
        size_t n = length < comms::MAX_PAYLOAD ? length : comms::MAX_PAYLOAD;
        remoteSend(comms::Type::Log, reinterpret_cast<const uint8_t*>(text), n);
        text += n;
        length -= n;
    }
#endif
}

bool remoteReceive(comms::Packet& packet) {
    if (!start()) return false;
    while (gRxTail != gRxHead) {
        uint8_t byte = gRx[gRxTail];
        gRxTail = (gRxTail + 1) & (RX_SIZE - 1);
        bool got = gDecoder.push(byte);
        gRemote.rxErrors = gDecoder.errors();
        if (got) {
            packet = gDecoder.packet();
            ++gRemote.received;
            return true;
        }
    }
    return false;
}

extern "C" void USART1_IRQHandler(void) {
    USART_TypeDef* u = USART1;
    uint32_t sr = u->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t byte = static_cast<uint8_t>(u->DR);  // reading SR then DR also clears ORE, NE and FE
        uint32_t next = (gRxHead + 1) & (RX_SIZE - 1);
        if (sr & USART_SR_ORE) ++gRemote.rxOverruns;
        if (next == gRxTail) ++gRemote.rxOverruns;
        else {
            gRx[gRxHead] = byte;
            gRxHead = next;
        }
    }
    if ((u->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (gTxTail != gTxHead) {
            u->DR = gTx[gTxTail];
            gTxTail = (gTxTail + 1) & (TX_SIZE - 1);
        }
        if (gTxTail == gTxHead) u->CR1 &= ~USART_CR1_TXEIE;
    }
}
