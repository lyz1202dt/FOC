#include "as5047p.hpp"

#include "spi.hpp"
#include "hardware.hpp"
#include "stm32g4xx_hal.h"

namespace {

constexpr uint16_t kParityBit = 0x8000U;
constexpr uint16_t kReadBit = 0x4000U;
constexpr uint16_t kErrorFlag = 0x4000U;
constexpr uint16_t kDataMask = 0x3FFFU;

constexpr uint16_t kErrflRegister = 0x0001U;
constexpr uint16_t kAngleComRegister = 0x3FFFU;

constexpr float kTwoPi = 6.2831853071795864769f;
constexpr float kCountsPerTurn = 16384.0f;

constexpr bsp::SPITransfer::TransferOptions kSpiOptions{
    SPI_DATASIZE_16BIT, SPI_POLARITY_LOW, SPI_PHASE_2EDGE, SPI_FIRSTBIT_MSB};

} // namespace

AS5047P::AS5047P() {
}

void AS5047P::setDmaAddress(uint16_t* tx_word, uint16_t* rx_word) {
    tx_dma_word_ = tx_word;
    rx_dma_word_ = rx_word;
}

bool AS5047P::init() {
    if (started_) {
        return true;
    }

    pending_response_ = Response::None;
    started_ = sensor_update();
    return started_;
}

bool AS5047P::sensor_update() {
    return start_exchange(build_read_command(kAngleComRegister), Response::Angle);
}

uint16_t AS5047P::get_raw_value() const {
    uint16_t value = 0U;
    __disable_irq();
    value = raw_value_;
    __enable_irq();
    return value;
}

uint16_t AS5047P::get_error_flags() const {
    uint16_t value = 0U;
    __disable_irq();
    value = error_flags_;
    __enable_irq();
    return value;
}

uint32_t AS5047P::get_error_count() const {
    uint32_t value = 0U;
    __disable_irq();
    value = error_count_;
    __enable_irq();
    return value;
}

void AS5047P::on_transfer_complete(void* ctx, bool success) {
    AS5047P* self = static_cast<AS5047P*>(ctx);

    self->in_flight_ = false;
    const Response response = self->active_response_;
    self->active_response_ = Response::None;

    bool request_error_flags = false;
    if (!success) {
        ++self->error_count_;
        self->pending_response_ = Response::None;
    } else {
        const uint16_t frame = *self->rxDmaWord();
        switch (response) {
        case Response::Angle:
            request_error_flags = !self->process_angle_frame(frame);
            break;
        case Response::ErrorFlags:
            (void)self->process_error_flags_frame(frame);
            break;
        case Response::None:
        default:
            break;
        }
    }

    uint16_t command = build_read_command(kAngleComRegister);
    Response command_response = Response::Angle;
    if (request_error_flags && self->pending_response_ != Response::ErrorFlags) {
        command = build_read_command(kErrflRegister);
        command_response = Response::ErrorFlags;
    }

    if (!self->start_exchange(command, command_response)) {
        self->started_ = false;
    }
}

uint16_t AS5047P::build_read_command(uint16_t address) {
    uint16_t frame = static_cast<uint16_t>(kReadBit | (address & kDataMask));
    if (!has_even_parity(frame)) {
        frame |= kParityBit;
    }
    return frame;
}

bool AS5047P::has_even_parity(uint16_t frame) {
    frame ^= static_cast<uint16_t>(frame >> 8);
    frame ^= static_cast<uint16_t>(frame >> 4);
    frame ^= static_cast<uint16_t>(frame >> 2);
    frame ^= static_cast<uint16_t>(frame >> 1);
    return (frame & 1U) == 0U;
}

uint16_t* AS5047P::txDmaWord() {
    return tx_dma_word_ != nullptr ? tx_dma_word_ : &tx_word_;
}

uint16_t* AS5047P::rxDmaWord() {
    return rx_dma_word_ != nullptr ? rx_dma_word_ : &rx_word_;
}

const uint16_t* AS5047P::rxDmaWord() const {
    return rx_dma_word_ != nullptr ? rx_dma_word_ : &rx_word_;
}

bool AS5047P::start_exchange(uint16_t command, Response command_response) {
    if (in_flight_) {
        return true;
    }

    const Response old_active = active_response_;
    const Response old_pending = pending_response_;
    const bool old_in_flight = in_flight_;

    uint16_t* tx_word = txDmaWord();
    uint16_t* rx_word = rxDmaWord();
    if (tx_word == nullptr || rx_word == nullptr) {
        return false;
    }

    *tx_word = command;
    *rx_word = 0U;
    active_response_ = pending_response_;
    pending_response_ = command_response;
    in_flight_ = true;

    transfer_.setDmaAddress(tx_word, rx_word);
    transfer_.size = 1U;
    transfer_.param = this;
    transfer_.options = kSpiOptions;
    transfer_.pre_transmit_cb = [](bsp::SPITransfer*) {
        BSP::sensor_spi_cs().write(false);
    };
    transfer_.transmited_cb = [](bsp::SPITransfer* transfer) {
        BSP::sensor_spi_cs().write(true);
        AS5047P::on_transfer_complete(
            transfer->param,
            transfer->status == HAL_OK && transfer->error_code == HAL_SPI_ERROR_NONE);
    };

    const bool accepted = BSP::sensor_spi_bus().transferDma(transfer_);

    if (!accepted) {
        active_response_ = old_active;
        pending_response_ = old_pending;
        in_flight_ = old_in_flight;
    }

    return accepted;
}

bool AS5047P::process_angle_frame(uint16_t frame) {
    if (!has_even_parity(frame) || (frame & kErrorFlag) != 0U) {
        ++error_count_;
        return false;
    }

    const uint16_t raw = static_cast<uint16_t>(frame & kDataMask);
    const float rad = static_cast<float>(raw) * kTwoPi / kCountsPerTurn;

    __disable_irq();
    raw_value_ = raw;
    __enable_irq();

    record_value(rad);
    return true;
}

bool AS5047P::process_error_flags_frame(uint16_t frame) {
    if (!has_even_parity(frame)) {
        ++error_count_;
        return false;
    }

    __disable_irq();
    error_flags_ = static_cast<uint16_t>(frame & kDataMask);
    __enable_irq();
    return true;
}
