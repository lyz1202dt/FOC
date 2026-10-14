#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "comm.hpp"
#include "ringbuffer.hpp"
#include "usart.hpp"

class UartComm : public Comm {
public:
    static constexpr std::size_t kMaxPayloadSize = 32U;
    static constexpr std::size_t kFrameHeadSize = 1U + 1U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;
    static constexpr std::size_t kCommandMaxSize = sizeof(uint32_t) + sizeof(uint32_t) + kMaxPayloadSize;
    static constexpr std::size_t kRxDmaBufferSize = 256U;
    static constexpr std::size_t kRxRingBufferSize = 512U;

    explicit UartComm(bsp::UartPort& port);

    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;

    bool init() override;
    bool set_dma_address(uint8_t* rx_buffer, std::size_t rx_size, uint8_t* tx_buffer, std::size_t tx_size);
    void register_recv_callback(std::function<void(void* data, int size)> callback) override;
    bool send_pack(void* data, int size) override;

private:
    static void uartTransmitCallback(bsp::UartTransmitEvent& event);
    static void uartReceiveCallback(bsp::UartReceiveEvent& event);

    bool startReceive();
    void onTransmitComplete();
    void onReceiveEvent(uint16_t position, bsp::Status status);

    void parseRxRing();
    bool tryParseFrame();

    bsp::UartPort* port_ = nullptr;
    std::function<void(void* data, int size)> recv_callback_;
    uint8_t rx_dma_buffer_storage_[kRxDmaBufferSize]{};
    uint8_t* rx_dma_buffer_ = rx_dma_buffer_storage_;
    std::size_t rx_dma_buffer_size_ = kRxDmaBufferSize;
    uint8_t rx_ring_buffer_[kRxRingBufferSize]{};
    tools::RingBuffer rx_ring_{rx_ring_buffer_, kRxRingBufferSize};
    uint8_t send_buffer_storage_[kFrameMaxSize]{};
    uint8_t* send_buffer_ = send_buffer_storage_;
    std::size_t send_buffer_size_ = kFrameMaxSize;
    alignas(uint32_t) uint8_t command_buffer_[kCommandMaxSize]{};
    volatile bool transmit_busy_ = false;
};
