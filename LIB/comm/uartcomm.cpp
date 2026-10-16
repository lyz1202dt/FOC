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

    const std::size_t payload_size = static_cast<std::size_t>(size) - sizeof(Command);
    const std::size_t frame_size = kFrameHeadSize + payload_size + kFrameCrcSize;
    const bool from_isr = __get_IPSR() != 0U;
    UBaseType_t interrupt_mask = 0U;
    if (from_isr) {
        interrupt_mask = portSET_INTERRUPT_MASK_FROM_ISR();
    } else {
        taskENTER_CRITICAL();
    }
    if (tx_descriptor_head_ - tx_descriptor_tail_ >= kTxDescriptorCount ||
        tx_used_ + frame_size > kTxRingSize) {
        if (from_isr) {
            portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);
        } else {
            taskEXIT_CRITICAL();
        }
        return false;
    }
    const std::size_t descriptor = tx_descriptor_head_ % kTxDescriptorCount;
    const std::size_t start = tx_head_;
    const std::size_t first_size = (kTxRingSize - start < frame_size) ? kTxRingSize - start : frame_size;
    uint8_t frame[kFrameMaxSize]{};
    frame[0] = kFrameHeader;
    frame[1] = static_cast<uint8_t>(payload_size);
    std::memcpy(frame + 2U, &command->cmd, sizeof(command->cmd));
    std::memcpy(frame + kFrameHeadSize, command->data, payload_size);
    const uint16_t crc = CalculateCrc(frame, frame_size - kFrameCrcSize);
    frame[frame_size - 2U] = static_cast<uint8_t>(crc);
    frame[frame_size - 1U] = static_cast<uint8_t>(crc >> 8U);
    std::memcpy(tx_ring_ + start, frame, first_size);
    if (first_size < frame_size) {
        std::memcpy(tx_ring_, frame + first_size, frame_size - first_size);
    }
    tx_descriptors_[descriptor] = {start, frame_size, first_size};
    tx_head_ = (start + frame_size) % kTxRingSize;
    tx_used_ += frame_size;
    ++tx_descriptor_head_;
    tx_segments_[descriptor][0] = {this, descriptor, first_size == frame_size};
    tx_segments_[descriptor][1] = {this, descriptor, true};
    bsp::UartTransmitEvent event = {};
    event.mode = tx_dma_buffer_ != nullptr ? bsp::UartTransmitMode::Dma : bsp::UartTransmitMode::Interrupt;
    event.data = tx_ring_ + start;
    event.size = static_cast<uint16_t>(first_size);
    event.context = &tx_segments_[descriptor][0];
    event.after_cb = &UartComm::TransmitComplete;
    if (from_isr) {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);
    } else {
        taskEXIT_CRITICAL();
    }
    if (port_.transmit(event) == bsp::Status::OK) {
        return true;
    }
    if (from_isr) {
        interrupt_mask = portSET_INTERRUPT_MASK_FROM_ISR();
    } else {
        taskENTER_CRITICAL();
    }
    tx_head_ = start;
    tx_used_ -= frame_size;
    --tx_descriptor_head_;
    if (from_isr) {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);
    } else {
        taskEXIT_CRITICAL();
    }
    return false;
}

void UartComm::TransmitComplete(bsp::UartTransmitEvent& event)
{
    auto* segment = static_cast<TxSegment*>(event.context);
    auto& comm = *segment->comm;
    const std::size_t descriptor = segment->descriptor;
    if (!segment->final_segment) {
        const auto& tx = comm.tx_descriptors_[descriptor];
        bsp::UartTransmitEvent next = {};
        next.mode = event.mode;
        next.data = comm.tx_ring_;
        next.size = static_cast<uint16_t>(tx.size - tx.first_size);
        next.context = &comm.tx_segments_[descriptor][1];
        next.after_cb = &UartComm::TransmitComplete;
        next.queue_front = true;
        (void)comm.port_.transmit(next);
        return;
    }
    const auto& tx = comm.tx_descriptors_[descriptor];
    const bool from_isr = __get_IPSR() != 0U;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        comm.tx_tail_ = (tx.start + tx.size) % kTxRingSize;
        comm.tx_used_ -= tx.size;
        ++comm.tx_descriptor_tail_;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        comm.tx_tail_ = (tx.start + tx.size) % kTxRingSize;
        comm.tx_used_ -= tx.size;
        ++comm.tx_descriptor_tail_;
        taskEXIT_CRITICAL();
    }
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
