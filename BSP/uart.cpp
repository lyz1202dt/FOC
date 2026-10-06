#include "uart.hpp"

#include <cstring>
#include <utility>

#include "pack.h"

namespace BSP {

namespace {
constexpr uint8_t kFrameHeader = 0x5AU;
constexpr std::size_t kHeaderOffset = 0U;
constexpr std::size_t kLengthOffset = 1U;
constexpr std::size_t kCommandOffset = 2U;
constexpr std::size_t kPayloadOffset = UARTPort::kFrameHeadSize;

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

UARTPort::UARTPort(bsp::UartPort& port)
    : port_(port)
{
}

void UARTPort::register_recv_callback(std::function<void(void* data, int size)> callback)
{
    recv_callback_ = std::move(callback);
}

bool UARTPort::init()
{
    return port_.init() && start_receive();
}

bool UARTPort::send_pack(void* data, int size)
{
    if (port_.native() == nullptr || data == nullptr || size < static_cast<int>(sizeof(Command))) {
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

    const HAL_StatusTypeDef status = port_.transmitDma(
        send_buffer_, frame_size, [this](const void*, uint16_t) {
            __disable_irq();
            transmit_busy_ = false;
            __enable_irq();
        });
    if (status != HAL_OK) {
        __disable_irq();
        transmit_busy_ = false;
        __enable_irq();
        return false;
    }

    return true;
}

bool UARTPort::start_receive()
{
    if (port_.native() == nullptr) {
        return false;
    }

    const HAL_StatusTypeDef status = port_.receiveToIdleDma(
        recv_buffer_,
        static_cast<uint16_t>(kFrameMaxSize),
        [this](const void*, uint16_t size) {
            on_receive(size);
        },
        true);
    return status == HAL_OK;
}

void UARTPort::on_receive(uint16_t size)
{
    if (HAL_UARTEx_GetRxEventType(port_.native()) == HAL_UART_RXEVENT_HT) {
        return;
    }

    uint16_t command_size = 0U;
    const bool valid = parse_received_frame(size, command_size);
    start_receive();

    if (valid && recv_callback_) {
        recv_callback_(command_buffer_, command_size);
    }
}

bool UARTPort::parse_received_frame(uint16_t size, uint16_t& command_size)
{
    if (size < kFrameHeadSize + kFrameCrcSize || size > kFrameMaxSize) {
        return false;
    }

    if (recv_buffer_[kHeaderOffset] != kFrameHeader) {
        return false;
    }

    const uint8_t payload_size = recv_buffer_[kLengthOffset];
    if (payload_size > kMaxPayloadSize) {
        return false;
    }

    const uint16_t expected_size = static_cast<uint16_t>(kFrameHeadSize + payload_size + kFrameCrcSize);
    if (size != expected_size) {
        return false;
    }

    const uint16_t received_crc = read_u16_le(&recv_buffer_[expected_size - kFrameCrcSize]);
    const uint16_t expected_crc = crc16_modbus(recv_buffer_, expected_size - kFrameCrcSize);
    if (received_crc != expected_crc) {
        return false;
    }

    auto* command = reinterpret_cast<Command*>(command_buffer_);
    command->cmd = read_u32_le(&recv_buffer_[kCommandOffset]);
    command->size = payload_size;
    if (payload_size > 0U) {
        std::memcpy(command->data, &recv_buffer_[kPayloadOffset], payload_size);
    }

    command_size = static_cast<uint16_t>(sizeof(Command) + payload_size);
    return true;
}

} // namespace BSP
