#include "uartcomm.hpp"

#include <cstring>
#include <utility>

#include "pack.h"

namespace {
constexpr uint8_t kFrameHeader = 0x5AU;
constexpr std::size_t kHeaderOffset = 0U;
constexpr std::size_t kLengthOffset = 1U;
constexpr std::size_t kCommandOffset = 2U;
constexpr std::size_t kPayloadOffset = UartComm::kFrameHeadSize;
constexpr std::size_t kMaxUartCommCount = 4U;

struct UartCommSlot {
    UART_HandleTypeDef* handle = nullptr;
    UartComm* comm = nullptr;
};

UartCommSlot g_uart_comm_slots[kMaxUartCommCount]{};

uint16_t crc16_modbus(const uint8_t* data, std::size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

uint16_t read_u16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

void write_u16_le(uint8_t* data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0x00FFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0x00FFU);
}

void write_u32_le(uint8_t* data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0x000000FFUL);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0x000000FFUL);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0x000000FFUL);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0x000000FFUL);
}
} // namespace

UartComm::UartComm(UART_HandleTypeDef& handle)
    : handle_(&handle)
{
}

void UartComm::register_recv_callback(std::function<void(void* data, int size)> callback)
{
    recv_callback_ = std::move(callback);
}

bool UartComm::init()
{
    if (handle_ == nullptr || !bind() || !configureReceiveDma()) {
        return false;
    }

    bool callbacks_registered = true;
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_COMPLETE_CB_ID, &UartComm::txCpltCallback) != HAL_OK) {
        callbacks_registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ERROR_CB_ID, &UartComm::errorCallback) != HAL_OK) {
        callbacks_registered = false;
    }
    if (HAL_UART_RegisterRxEventCallback(handle_, &UartComm::rxEventCallback) != HAL_OK) {
        callbacks_registered = false;
    }

    rx_dma_last_pos_ = 0U;
    rx_ring_head_ = 0U;
    rx_ring_tail_ = 0U;
    rx_ring_size_ = 0U;
    transmit_busy_ = false;

    return callbacks_registered && startReceive();
}

bool UartComm::send_pack(void* data, int size)
{
    if (handle_ == nullptr || data == nullptr || size < static_cast<int>(sizeof(Command))) {
        return false;
    }

    const auto* command = static_cast<const Command*>(data);
    const uint32_t payload_size = command->size;
    if (payload_size > kMaxPayloadSize || size != static_cast<int>(sizeof(Command) + payload_size)) {
        return false;
    }

    bool accepted = false;
    __disable_irq();
    if (!transmit_busy_) {
        transmit_busy_ = true;
        accepted = true;
    }
    __enable_irq();

    if (!accepted) {
        return false;
    }

    const uint16_t frame_size = static_cast<uint16_t>(kFrameHeadSize + payload_size + kFrameCrcSize);
    send_buffer_[kHeaderOffset] = kFrameHeader;
    send_buffer_[kLengthOffset] = static_cast<uint8_t>(payload_size);
    write_u32_le(&send_buffer_[kCommandOffset], command->cmd);
    if (payload_size > 0U) {
        std::memcpy(&send_buffer_[kPayloadOffset], command->data, payload_size);
    }

    const uint16_t crc = crc16_modbus(send_buffer_, frame_size - kFrameCrcSize);
    write_u16_le(&send_buffer_[frame_size - kFrameCrcSize], crc);

    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(handle_, send_buffer_, frame_size);
    if (status != HAL_OK) {
        __disable_irq();
        transmit_busy_ = false;
        __enable_irq();
        return false;
    }

    return true;
}

void UartComm::txCpltCallback(UART_HandleTypeDef* huart)
{
    UartComm* comm = findInstance(huart);
    if (comm != nullptr) {
        comm->onTransmitComplete();
    }
}

void UartComm::rxEventCallback(UART_HandleTypeDef* huart, uint16_t position)
{
    UartComm* comm = findInstance(huart);
    if (comm != nullptr) {
        comm->onReceiveEvent(position);
    }
}

void UartComm::errorCallback(UART_HandleTypeDef* huart)
{
    UartComm* comm = findInstance(huart);
    if (comm != nullptr) {
        comm->onError();
    }
}

UartComm* UartComm::findInstance(UART_HandleTypeDef* huart)
{
    for (UartCommSlot& slot : g_uart_comm_slots) {
        if (slot.handle == huart) {
            return slot.comm;
        }
    }
    return nullptr;
}

bool UartComm::bind()
{
    for (UartCommSlot& slot : g_uart_comm_slots) {
        if (slot.handle == handle_) {
            slot.comm = this;
            return true;
        }
    }

    for (UartCommSlot& slot : g_uart_comm_slots) {
        if (slot.handle == nullptr) {
            slot.handle = handle_;
            slot.comm = this;
            return true;
        }
    }

    return false;
}

bool UartComm::configureReceiveDma() const
{
    if (handle_->hdmarx == nullptr) {
        return false;
    }

    if (handle_->hdmarx->Init.Mode == DMA_CIRCULAR) {
        return true;
    }

    handle_->hdmarx->Init.Mode = DMA_CIRCULAR;
    return HAL_DMA_Init(handle_->hdmarx) == HAL_OK;
}

bool UartComm::startReceive()
{
    if (handle_ == nullptr || handle_->hdmarx == nullptr) {
        return false;
    }

    rx_dma_last_pos_ = 0U;
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(handle_, rx_dma_buffer_, static_cast<uint16_t>(kRxDmaBufferSize));
    return status == HAL_OK;
}

void UartComm::onTransmitComplete()
{
    __disable_irq();
    transmit_busy_ = false;
    __enable_irq();
}

void UartComm::onReceiveEvent(uint16_t position)
{
    std::size_t current_pos = position;
    if (current_pos > kRxDmaBufferSize) {
        current_pos = kRxDmaBufferSize;
    }

    if (current_pos > rx_dma_last_pos_) {
        pushDmaRange(rx_dma_last_pos_, current_pos);
    } else if (current_pos < rx_dma_last_pos_) {
        pushDmaRange(rx_dma_last_pos_, kRxDmaBufferSize);
        pushDmaRange(0U, current_pos);
    }

    rx_dma_last_pos_ = current_pos;
    parseRxRing();
}

void UartComm::onError()
{
    if (handle_ != nullptr && handle_->gState == HAL_UART_STATE_READY) {
        __disable_irq();
        transmit_busy_ = false;
        __enable_irq();
    }

    if (handle_ != nullptr && handle_->RxState == HAL_UART_STATE_READY) {
        (void)startReceive();
    }
}

void UartComm::pushDmaRange(std::size_t begin, std::size_t end)
{
    for (std::size_t i = begin; i < end; ++i) {
        pushRxByte(rx_dma_buffer_[i]);
    }
}

void UartComm::pushRxByte(uint8_t value)
{
    rx_ring_buffer_[rx_ring_head_] = value;
    rx_ring_head_ = (rx_ring_head_ + 1U) % kRxRingBufferSize;

    if (rx_ring_size_ == kRxRingBufferSize) {
        rx_ring_tail_ = (rx_ring_tail_ + 1U) % kRxRingBufferSize;
    } else {
        ++rx_ring_size_;
    }
}

uint8_t UartComm::peekRxByte(std::size_t offset) const
{
    return rx_ring_buffer_[(rx_ring_tail_ + offset) % kRxRingBufferSize];
}

void UartComm::dropRxBytes(std::size_t count)
{
    if (count > rx_ring_size_) {
        count = rx_ring_size_;
    }

    rx_ring_tail_ = (rx_ring_tail_ + count) % kRxRingBufferSize;
    rx_ring_size_ -= count;
}

void UartComm::parseRxRing()
{
    while (tryParseFrame()) {
    }
}

bool UartComm::tryParseFrame()
{
    constexpr std::size_t min_frame_size = kFrameHeadSize + kFrameCrcSize;

    while (rx_ring_size_ >= min_frame_size && peekRxByte(0U) != kFrameHeader) {
        dropRxBytes(1U);
    }

    if (rx_ring_size_ < min_frame_size) {
        return false;
    }

    const uint8_t payload_size = peekRxByte(kLengthOffset);
    if (payload_size > kMaxPayloadSize) {
        dropRxBytes(1U);
        return true;
    }

    const std::size_t frame_size = kFrameHeadSize + payload_size + kFrameCrcSize;
    if (rx_ring_size_ < frame_size) {
        return false;
    }

    uint8_t frame[kFrameMaxSize]{};
    for (std::size_t i = 0U; i < frame_size; ++i) {
        frame[i] = peekRxByte(i);
    }

    const uint16_t received_crc = read_u16_le(&frame[frame_size - kFrameCrcSize]);
    const uint16_t expected_crc = crc16_modbus(frame, frame_size - kFrameCrcSize);
    if (received_crc != expected_crc) {
        dropRxBytes(1U);
        return true;
    }

    auto* command = reinterpret_cast<Command*>(command_buffer_);
    command->cmd = read_u32_le(&frame[kCommandOffset]);
    command->size = payload_size;
    if (payload_size > 0U) {
        std::memcpy(command->data, &frame[kPayloadOffset], payload_size);
    }

    dropRxBytes(frame_size);

    if (recv_callback_) {
        recv_callback_(command_buffer_, static_cast<int>(sizeof(Command) + payload_size));
    }

    return rx_ring_size_ >= min_frame_size;
}
