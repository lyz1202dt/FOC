#include "as5600.hpp"

namespace {

constexpr uint16_t kAs5600Address = 0x36U << 1;
constexpr uint16_t kStatusRegister = 0x0BU;
constexpr uint8_t kMagnetDetected = 1U << 5;
constexpr uint8_t kMagnetTooWeak = 1U << 4;
constexpr uint8_t kMagnetTooStrong = 1U << 3;
constexpr uint8_t kStatusValid = 1U << 7;
constexpr float kRadiansPerCount = 2.0f * 3.14159265358979323846f / 4096.0f;

static_assert(std::atomic<bool>::is_always_lock_free);
static_assert(std::atomic<uint8_t>::is_always_lock_free);

}  // namespace

AS5600::AS5600(Flash& flash, BSP::I2CBus& bus, bool use_angle_when_magnet_weak)
    : Encoder(flash), bus_(bus),
      use_angle_when_magnet_weak_(use_angle_when_magnet_weak) {
    // STATUS, RAW ANGLE and ANGLE are adjacent; read all five bytes at once.
    transfer_.operation = bsp::I2COperation::MemoryRead;
    transfer_.device_address = kAs5600Address;
    transfer_.memory_address = kStatusRegister;
    transfer_.memory_address_size = I2C_MEMADD_SIZE_8BIT;
    transfer_.rx_data = recv_buffer_;
    transfer_.size = sizeof(recv_buffer_);
    transfer_.param = this;
    transfer_.transmited_cb = [](bsp::I2CTransfer* transfer) {
        AS5600::on_read_complete(transfer->param,
            transfer->status == HAL_OK && transfer->error_code == HAL_I2C_ERROR_NONE);
    };
}

bool AS5600::init() {
    return read_pending_.load(std::memory_order_acquire) || sensor_update();
}

bool AS5600::sensor_update() {
    bool expected = false;
    if (!read_pending_.compare_exchange_strong(expected, true,
                                               std::memory_order_acq_rel)) {
        return false;
    }

    if (!bus_.transferIt(transfer_)) {
        status_.store(0U, std::memory_order_release);
        read_pending_.store(false, std::memory_order_release);
        return false;
    }
    return true;
}

bool AS5600::GetMagnetStatus(MagnetStatus& status) const {
    const uint8_t value = status_.load(std::memory_order_acquire);
    if ((value & kStatusValid) == 0U) {
        return false;
    }
    status.detected = (value & kMagnetDetected) != 0U;
    status.too_weak = (value & kMagnetTooWeak) != 0U;
    status.too_strong = (value & kMagnetTooStrong) != 0U;
    return true;
}

void AS5600::on_read_complete(void* ctx, bool success) {
    auto* self = static_cast<AS5600*>(ctx);
    if (success) {
        const uint8_t status = self->recv_buffer_[0];
        // ANGLE is at offsets 3 and 4; mask the unused high nibble.
        const uint16_t angle_count =
            (static_cast<uint16_t>(self->recv_buffer_[3] & 0x0FU) << 8) |
            static_cast<uint16_t>(self->recv_buffer_[4]);
        if ((status & kMagnetDetected) != 0U &&
            (status & kMagnetTooStrong) == 0U &&
            (self->use_angle_when_magnet_weak_ || (status & kMagnetTooWeak) == 0U)) {
            self->record_value(static_cast<float>(angle_count) * kRadiansPerCount);
        }
        self->status_.store(status | kStatusValid, std::memory_order_release);
    } else {
        self->status_.store(0U, std::memory_order_release);
    }

    // Release ownership only after consuming the receive buffer. Retry errors
    // explicitly from the task to avoid recursion on immediate HAL failures.
    self->read_pending_.store(false, std::memory_order_release);
    if (success) {
        (void)self->sensor_update();
    }
}
