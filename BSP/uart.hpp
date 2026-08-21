#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "comm.hpp"
#include "usart.h"

namespace BSP {

class UARTPort : public Comm {
public:
    static constexpr std::size_t kMaxPayloadSize = 120U;
    static constexpr std::size_t kFrameHeadSize  = 1U + 1U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize   = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize   = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;
    static constexpr std::size_t kCommandMaxSize = sizeof(uint32_t) + sizeof(uint32_t) + kMaxPayloadSize;

    explicit UARTPort(UART_HandleTypeDef* huart);

    UARTPort(const UARTPort&)            = delete;
    UARTPort& operator=(const UARTPort&) = delete;

    bool init();
    void register_recv_callback(std::function<void(void* data, int size)> callback) override;
    bool send_pack(void* data, int size) override;

    void on_error_irq(UART_HandleTypeDef* huart);
    void on_transmit_cplt_irq(UART_HandleTypeDef* huart);
    void on_receive_cplt_irq(UART_HandleTypeDef* huart, uint16_t size);

private:
    bool owns(UART_HandleTypeDef* huart) const;
    bool start_receive();
    bool parse_received_frame(uint16_t size, uint16_t& command_size);

    UART_HandleTypeDef* huart_;
    std::function<void(void* data, int size)> recv_callback_;
    uint8_t recv_buffer_[kFrameMaxSize]{};
    uint8_t send_buffer_[kFrameMaxSize]{};
    alignas(uint32_t) uint8_t command_buffer_[kCommandMaxSize]{};
    bool transmit_busy_{false};
};

/*数据包结构：
0x5A(1字节包头)|0x00(1字节包长度)|0x00000001(4字节命令字)|...若干数据，不超过120字节|0xAAAA(2字节CRC校验)
*/

} // namespace BSP
