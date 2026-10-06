#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "comm.hpp"
#include "stm32g4xx_hal.h"

class UartComm : public Comm {
public:
    static constexpr std::size_t kMaxPayloadSize = 120U;
    static constexpr std::size_t kFrameHeadSize = 1U + 1U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;
    static constexpr std::size_t kCommandMaxSize = sizeof(uint32_t) + sizeof(uint32_t) + kMaxPayloadSize;
    static constexpr std::size_t kRxDmaBufferSize = 256U;
    static constexpr std::size_t kRxRingBufferSize = 512U;

    explicit UartComm(UART_HandleTypeDef& handle);

    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;

    bool init();
    void register_recv_callback(std::function<void(void* data, int size)> callback) override;
    bool send_pack(void* data, int size) override;

private:
    static void txCpltCallback(UART_HandleTypeDef* huart);
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t position);
    static void errorCallback(UART_HandleTypeDef* huart);

    static UartComm* findInstance(UART_HandleTypeDef* huart);

    bool bind();
    bool configureReceiveDma() const;
    bool startReceive();
    void onTransmitComplete();
    void onReceiveEvent(uint16_t position);
    void onError();

    void pushDmaRange(std::size_t begin, std::size_t end);
    void pushRxByte(uint8_t value);
    uint8_t peekRxByte(std::size_t offset) const;
    void dropRxBytes(std::size_t count);
    void parseRxRing();
    bool tryParseFrame();

    UART_HandleTypeDef* handle_ = nullptr;
    std::function<void(void* data, int size)> recv_callback_;
    uint8_t rx_dma_buffer_[kRxDmaBufferSize]{};
    uint8_t rx_ring_buffer_[kRxRingBufferSize]{};
    std::size_t rx_dma_last_pos_ = 0U;
    std::size_t rx_ring_head_ = 0U;
    std::size_t rx_ring_tail_ = 0U;
    std::size_t rx_ring_size_ = 0U;
    uint8_t send_buffer_[kFrameMaxSize]{};
    alignas(uint32_t) uint8_t command_buffer_[kCommandMaxSize]{};
    bool transmit_busy_ = false;
};
