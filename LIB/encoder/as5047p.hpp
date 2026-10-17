#pragma once

#include "encoder.hpp"
#include "spi.hpp"

#include <atomic>
#include <cstdint>

class AS5047P : public Encoder {
public:
    explicit AS5047P(Flash &flash);
    // Call once from a task with the sensor SPI bus initialized and idle.
    // The encoder and its buffers must outlive all queued transfers.
    bool init() override;
    // Return the cached radians, then request one nonblocking SPI frame.
    float get_value() override;

private:
    enum class Response : uint8_t { None, Angle, ErrorFlags };

    bsp::SpiTransferEvent transfer_{};
    uint16_t tx_word_{0U};
    uint16_t rx_word_{0U};
    uint16_t next_command_{0xFFFFU};
    Response pending_response_{Response::None};
    // Owns the buffers and pipeline state until the completion callback releases it.
    std::atomic<bool> read_pending_{false};
    std::atomic<bool> initialized_{false};
};
