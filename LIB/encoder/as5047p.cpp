#include "as5047p.hpp"

namespace {

// Read bit plus register address, with even parity across all 16 bits.
constexpr uint16_t kReadAngleCommand = 0xFFFFU;  // ANGLECOM (0x3FFF).
constexpr uint16_t kReadErrorCommand = 0x4001U;  // ERRFL (0x0001), clears errors.
constexpr uint16_t kErrorFlag = 0x4000U;
constexpr uint16_t kAngleMask = 0x3FFFU;
constexpr float kRadiansPerCount = 6.2831853071795864769f / 16384.0f;

static_assert(std::atomic<bool>::is_always_lock_free);

}  // namespace

AS5047P::AS5047P(Flash& flash, BSP::SPIBus& bus, BSP::Gpio& cs)
    : Encoder(flash), bus_(bus), cs_(cs) {
    transfer_.mode = bsp::SpiTransferMode::Interrupt;
    transfer_.tx_data = &tx_word_;
    transfer_.rx_data = &rx_word_;
    transfer_.size = 1U;
    transfer_.context = this;
    // SPI hardware configuration comes from CubeMX; transfers leave it unchanged.
    transfer_.before_cb = [](bsp::SpiTransferEvent& event) {
        auto* self = static_cast<AS5047P*>(event.context);
        self->cs_.write(false);
        // tL >= 350 ns, even at the STM32G431 maximum clock of 170 MHz.
        __asm volatile(".rept 64\n\tnop\n\t.endr" ::: "memory");
    };
    transfer_.after_cb = [](bsp::SpiTransferEvent& event) {
        auto* self = static_cast<AS5047P*>(event.context);
        self->cs_.write(true);
        // tCSn >= 350 ns before another caller may submit the next frame.
        __asm volatile(".rept 64\n\tnop\n\t.endr" ::: "memory");
        if (event.status == bsp::SpiTransferStatus::Ok) {
            uint16_t parity = self->rx_word_;
            parity ^= static_cast<uint16_t>(parity >> 8);
            parity ^= static_cast<uint16_t>(parity >> 4);
            parity ^= static_cast<uint16_t>(parity >> 2);
            parity ^= static_cast<uint16_t>(parity >> 1);
            const bool valid_frame = (parity & 1U) == 0U &&
                                    (self->rx_word_ & kErrorFlag) == 0U;
            self->next_command_ = kReadAngleCommand;
            if (self->pending_response_ == Response::Angle) {
                if (valid_frame) {
                    self->record_value(static_cast<float>(self->rx_word_ & kAngleMask) *
                                       kRadiansPerCount);
                } else if (self->tx_word_ != kReadErrorCommand) {
                    self->next_command_ = kReadErrorCommand;
                }
            }
            // MISO belongs to the preceding command, not the command just sent.
            // Discard the first response and never interpret ERRFL as an angle.
            self->pending_response_ = self->tx_word_ == kReadAngleCommand
                                          ? Response::Angle : Response::ErrorFlags;
        } else {
            // A partial frame loses pipeline alignment. Clear errors and prime again.
            self->pending_response_ = Response::None;
            self->next_command_ = kReadErrorCommand;
        }
        // No automatic resubmission: get_value() controls the sampling rate.
        self->read_pending_.store(false, std::memory_order_release);
    };
}

bool AS5047P::init() {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    if (!bus_.valid()) {
        return false;
    }

    cs_.write(true);
    initialized_.store(true, std::memory_order_release);
    return true;
}

float AS5047P::get_value() {
    const float rad = Encoder::get_value();
    if (!initialized_.load(std::memory_order_acquire)) {
        return rad;
    }
    bool expected = false;
    if (!read_pending_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return rad;
    }

    tx_word_ = next_command_;
    if (!bus_.transfer(transfer_)) {
        // Queue rejection sends no frame, so keep the existing pipeline state.
        read_pending_.store(false, std::memory_order_release);
    }
    return rad;
}
