#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "comm.hpp"
#include "usart.hpp"

class UartComm : public Comm {
public:
    // Command header and payload together occupy at most 64 bytes.
    static constexpr std::size_t kCommandMaxSize = 64U;
    static constexpr std::size_t kMaxPayloadSize = kCommandMaxSize - 2U * sizeof(uint32_t);
    static constexpr std::size_t kFrameHeadSize = 2U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;

    explicit UartComm(bsp::UartPort& port);

    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;

    bool init() override;
    // Configure before init to select DMA for TX/RX. TX needs kFrameMaxSize bytes;
    // RX uses at most kFrameMaxSize bytes per completion, even with a larger buffer.
    bool set_dma_address(uint8_t* rx_buffer, std::size_t rx_size, uint8_t* tx_buffer, std::size_t tx_size);
    // Callback runs in IRQ context; received command is valid only during callback.
    void register_recv_callback(std::function<void(void* data, int size)> callback) override;
    // Queues the caller's pointer without copying. Keep the command valid and
    // unchanged until its queued transfer starts and PrepareTransmit frames it.
    bool send_pack(void* data, int size) override;

private:
    static uint16_t CalculateCrc(const uint8_t* data, std::size_t size);
    static void PrepareTransmit(bsp::UartTransmitEvent& event);
    static void ReceiveComplete(bsp::UartReceiveEvent& event);

    bool StartReceive();
    void ProcessReceive(const uint8_t* data, std::size_t size);

    bsp::UartPort& port_;
    std::function<void(void* data, int size)> recv_callback_;
    uint8_t* rx_dma_buffer_ = nullptr;
    uint8_t* tx_dma_buffer_ = nullptr;
    uint16_t rx_dma_size_ = 0U;
    uint8_t rx_byte_ = 0U;
    uint8_t tx_frame_[kFrameMaxSize]{};
    uint8_t rx_frame_[kFrameMaxSize]{};
    std::size_t rx_frame_size_ = 0U;
    bool initialized_ = false;
    bool receive_rearming_ = false;
};
