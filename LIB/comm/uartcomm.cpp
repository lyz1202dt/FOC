#include "uartcomm.hpp"

#include <cstring>
#include <utility>

#include "pack.h"

namespace {
constexpr uint8_t kFrameHeader = 0x5AU;
} // namespace

uint16_t UartComm::CalculateCrc(const uint8_t* data, std::size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = static_cast<uint16_t>((crc >> 1U) ^ ((crc & 1U) != 0U ? 0xA001U : 0U));
        }
    }
    return crc;
}

UartComm::UartComm(bsp::UartPort& port)
    : port_(port)
{
}

void UartComm::register_recv_callback(std::function<void(void* data, int size)> callback)
{
    recv_callback_ = std::move(callback);
}

bool UartComm::set_dma_address(uint8_t* rx_buffer, std::size_t rx_size, uint8_t* tx_buffer, std::size_t tx_size)
{
    if (initialized_ || rx_buffer == nullptr || rx_size == 0U || tx_buffer == nullptr ||
        tx_size < kFrameMaxSize) {
        return false;
    }
    rx_dma_buffer_ = rx_buffer;
    rx_dma_size_ = static_cast<uint16_t>(rx_size < kFrameMaxSize ? rx_size : kFrameMaxSize);
    tx_dma_buffer_ = tx_buffer;
    return true;
}

bool UartComm::init()
{
    if (initialized_ || port_.native() == nullptr) {
        return false;
    }
    if (tx_dma_buffer_ != nullptr) {
        auto* handle = port_.native();
        if (handle->hdmarx == nullptr || handle->hdmatx == nullptr) {
            return false;
        }
        // Receive-to-idle completion consumes and rearms a normal DMA buffer.
        if (handle->hdmarx->Init.Mode != DMA_NORMAL) {
            handle->hdmarx->Init.Mode = DMA_NORMAL;
            if (HAL_DMA_Init(handle->hdmarx) != HAL_OK) {
                return false;
            }
        }
    }
    if (!port_.init()) {
        return false;
    }
    initialized_ = true;
    return StartReceive();
}

bool UartComm::send_pack(void* data, int size)
{
    if (!initialized_ || data == nullptr || size < static_cast<int>(sizeof(Command)) ||
        size > static_cast<int>(kCommandMaxSize)) {
        return false;
    }
    const auto* command = static_cast<const Command*>(data);
    if (command->size != static_cast<uint32_t>(size - sizeof(Command))) {
        return false;
    }

    bsp::UartTransmitEvent event = {};
    event.mode = tx_dma_buffer_ != nullptr ? bsp::UartTransmitMode::Dma : bsp::UartTransmitMode::Interrupt;
    event.data = data;
    event.size = static_cast<uint16_t>(size);
    event.context = this;
    event.before_cb = &UartComm::PrepareTransmit;
    return port_.transmit(event) == bsp::Status::OK;
}

void UartComm::PrepareTransmit(bsp::UartTransmitEvent& event)
{
    auto& comm = *static_cast<UartComm*>(event.context);
    const auto* command = static_cast<const Command*>(event.data);
    uint8_t* frame = event.mode == bsp::UartTransmitMode::Dma ? comm.tx_dma_buffer_ : comm.tx_frame_;
    // Use the validated descriptor length, not mutable caller metadata.
    const std::size_t payload_size = event.size - sizeof(Command);
    const uint32_t command_id = command->cmd;
    std::memmove(frame + kFrameHeadSize, command->data, payload_size);
    frame[0] = kFrameHeader;
    frame[1] = static_cast<uint8_t>(payload_size);
    // STM32 and the wire command ID are both little endian.
    std::memcpy(frame + 2U, &command_id, sizeof(command_id));
    const uint16_t crc = CalculateCrc(frame, event.size - kFrameCrcSize);
    frame[event.size - 2U] = static_cast<uint8_t>(crc);
    frame[event.size - 1U] = static_cast<uint8_t>(crc >> 8U);
    event.data = frame;
}

bool UartComm::StartReceive()
{
    bsp::UartReceiveEvent event = {};
    event.mode = rx_dma_buffer_ != nullptr ? bsp::UartReceiveMode::ToIdleDma : bsp::UartReceiveMode::Interrupt;
    event.data = rx_dma_buffer_ != nullptr ? rx_dma_buffer_ : &rx_byte_;
    event.size = rx_dma_buffer_ != nullptr ? rx_dma_size_ : 1U;
    event.context = this;
    event.after_cb = &UartComm::ReceiveComplete;
    return port_.receive(event) == bsp::Status::OK;
}

void UartComm::ReceiveComplete(bsp::UartReceiveEvent& event)
{
    auto& comm = *static_cast<UartComm*>(event.context);
    if (event.status == bsp::Status::OK) {
        comm.ProcessReceive(static_cast<const uint8_t*>(event.data), event.transferred_size);
    } else {
        comm.rx_frame_size_ = 0U;
    }
    // Do not retry a synchronous HAL start failure recursively.
    if (__get_IPSR() != 0U && !comm.receive_rearming_) {
        comm.receive_rearming_ = true;
        (void)comm.StartReceive();
        comm.receive_rearming_ = false;
    }
}

void UartComm::ProcessReceive(const uint8_t* data, std::size_t size)
{
    for (std::size_t i = 0U; i < size; ++i) {
        rx_frame_[rx_frame_size_++] = data[i];
        while (rx_frame_size_ >= 2U) {
            std::size_t consumed = 1U;
            if (rx_frame_[0] == kFrameHeader && rx_frame_[1] <= kMaxPayloadSize) {
                const std::size_t frame_size = kFrameHeadSize + rx_frame_[1] + kFrameCrcSize;
                if (rx_frame_size_ < frame_size) {
                    break;
                }
                const uint16_t crc = static_cast<uint16_t>(rx_frame_[frame_size - 2U]) |
                                     static_cast<uint16_t>(rx_frame_[frame_size - 1U] << 8U);
                if (crc == CalculateCrc(rx_frame_, frame_size - kFrameCrcSize)) {
                    // Build the callback's Command on the stack instead of retaining
                    // a second command buffer. Its lifetime ends with the callback.
                    alignas(uint32_t) uint8_t command_data[kCommandMaxSize];
                    auto* command = reinterpret_cast<Command*>(command_data);
                    std::memcpy(&command->cmd, rx_frame_ + 2U, sizeof(command->cmd));
                    command->size = rx_frame_[1];
                    std::memcpy(command->data, rx_frame_ + kFrameHeadSize, command->size);
                    consumed = frame_size;
                    if (recv_callback_) {
                        recv_callback_(command_data, static_cast<int>(sizeof(Command) + command->size));
                    }
                }
            }
            rx_frame_size_ -= consumed;
            std::memmove(rx_frame_, rx_frame_ + consumed, rx_frame_size_);
        }
    }
}
