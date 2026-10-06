#pragma once

#include "encoder.hpp"
#include "spi.hpp"

#include <cstdint>

class AS5047P : public Encoder {
public:
    AS5047P();
    bool init() override;
    void setDmaAddress(uint16_t* tx_word, uint16_t* rx_word);
    bool sensor_update();

    uint16_t get_raw_value() const;
    uint16_t get_error_flags() const;
    uint32_t get_error_count() const;

private:
    enum class Response : uint8_t {
        None,
        Angle,
        ErrorFlags,
    };

    static void on_transfer_complete(void* ctx, bool success);
    static uint16_t build_read_command(uint16_t address);
    static bool has_even_parity(uint16_t frame);

    uint16_t* txDmaWord();
    uint16_t* rxDmaWord();
    const uint16_t* rxDmaWord() const;

    bool start_exchange(uint16_t command, Response command_response);
    bool process_angle_frame(uint16_t frame);
    bool process_error_flags_frame(uint16_t frame);

    uint16_t tx_word_{0U};
    uint16_t rx_word_{0U};
    uint16_t* tx_dma_word_{nullptr};
    uint16_t* rx_dma_word_{nullptr};
    bsp::SPITransfer transfer_{};
    Response active_response_{Response::None};
    Response pending_response_{Response::None};
    bool started_{false};
    bool in_flight_{false};

    uint16_t raw_value_{0U};
    uint16_t error_flags_{0U};
    uint32_t error_count_{0U};
};
