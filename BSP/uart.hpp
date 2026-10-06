#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "comm.hpp"
#include "usart.hpp"

namespace BSP {

class UARTPort : public Comm {
public:
    static constexpr std::size_t kMaxPayloadSize = 120U;
    static constexpr std::size_t kFrameHeadSize = 1U + 1U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;
    static constexpr std::size_t kCommandMaxSize = sizeof(uint32_t) + sizeof(uint32_t) + kMaxPayloadSize;

    explicit UARTPort(bsp::UartPort& port);

    UARTPort(const UARTPort&) = delete;
    UARTPort& operator=(const UARTPort&) = delete;

    bool init();
    void register_recv_callback(std::function<void(void* data, int size)> callback) override;
    bool send_pack(void* data, int size) override;

private:
    bool start_receive();
    void on_receive(uint16_t size);
    bool parse_received_frame(uint16_t size, uint16_t& command_size);

    bsp::UartPort& port_;
    std::function<void(void* data, int size)> recv_callback_;
    uint8_t recv_buffer_[kFrameMaxSize]{};
    uint8_t send_buffer_[kFrameMaxSize]{};
    alignas(uint32_t) uint8_t command_buffer_[kCommandMaxSize]{};
    bool transmit_busy_{false};
};

} // namespace BSP
