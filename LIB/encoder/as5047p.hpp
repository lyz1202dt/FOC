#pragma once

#include "encoder.hpp"
#include "gpio.hpp"
#include "spi.hpp"

#include <atomic>
#include <cstdint>

class AS5047P : public Encoder {
public:
    AS5047P(Flash& flash, BSP::SPIBus& bus, BSP::Gpio& cs);
    bool init() override;
    float get_value() override;

private:
    enum class Response : uint8_t { None, Angle, ErrorFlags };

    BSP::SPIBus& bus_;
    BSP::Gpio& cs_;
    bsp::SpiTransferEvent transfer_{};
    uint16_t tx_word_{0U};
    uint16_t rx_word_{0U};
    uint16_t next_command_{0xFFFFU};
    Response pending_response_{Response::None};
    // Owns the buffers and pipeline state until the completion callback releases it.
    std::atomic<bool> read_pending_{false};
    std::atomic<bool> initialized_{false};
};
