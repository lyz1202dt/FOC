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
    void register_recv_callback(std::function<void(Command* data)> callback) override;
    // Copies and queues the command; the caller may release its storage on return.
    bool send_pack(Command* data) override;

private:
    static uint16_t CalculateCrc(const uint8_t* data, std::size_t size);
    static void TransmitComplete(bsp::UartTransmitEvent& event);
    static void ReceiveComplete(bsp::UartReceiveEvent& event);

    bool StartReceive();
    void ProcessReceive(const uint8_t* data, std::size_t size);

    struct TxDescriptor {
        std::size_t start = 0U;
        std::size_t size = 0U;
        std::size_t first_size = 0U;
    };
    struct TxSegment {
        UartComm* comm = nullptr;
        std::size_t descriptor = 0U;
        bool final_segment = false;
    };

    static constexpr std::size_t kTxRingSize = 512U;
    static constexpr std::size_t kTxDescriptorCount = 8U;

    bsp::UartPort& port_;
    std::function<void(Command* data)> recv_callback_;
    uint8_t* rx_dma_buffer_ = nullptr;
    uint8_t* tx_dma_buffer_ = nullptr;
    uint16_t rx_dma_size_ = 0U;
    uint8_t rx_byte_ = 0U;
    uint8_t tx_ring_[kTxRingSize]{};
    TxDescriptor tx_descriptors_[kTxDescriptorCount]{};
    TxSegment tx_segments_[kTxDescriptorCount][2]{};
    std::size_t tx_head_ = 0U;
    std::size_t tx_tail_ = 0U;
    std::size_t tx_descriptor_head_ = 0U;
    std::size_t tx_descriptor_tail_ = 0U;
    std::size_t tx_used_ = 0U;
    uint8_t rx_frame_[kFrameMaxSize]{};
    std::size_t rx_frame_size_ = 0U;
    bool initialized_ = false;
    bool receive_rearming_ = false;
};
