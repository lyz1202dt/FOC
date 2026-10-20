#include "as5600.hpp"
#include "hardware.hpp"

namespace {

constexpr uint16_t kAs5600Address = 0x36U << 1;
constexpr uint16_t kStatusRegister = 0x0BU;
constexpr uint8_t kMagnetDetected = 1U << 5;
constexpr uint8_t kMagnetTooWeak = 1U << 4;
constexpr uint8_t kMagnetTooStrong = 1U << 3;
constexpr uint8_t kStatusValid = 1U << 7;
constexpr float kRadiansPerCount = 2.0f * 3.14159265358979323846f / 4096.0f;
constexpr uint32_t kReadTimeoutUs = 10000U;

static_assert(std::atomic<bool>::is_always_lock_free);
static_assert(std::atomic<uint8_t>::is_always_lock_free);
static_assert(std::atomic<uint32_t>::is_always_lock_free);

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
        auto* self = static_cast<AS5600*>(transfer->param);
        if (transfer->status == HAL_OK && transfer->error_code == HAL_I2C_ERROR_NONE) {
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
            // NACK is a completed transaction; BUSY/bus errors need peripheral recovery.
            if (transfer->status == HAL_BUSY ||
                (transfer->error_code & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO |
                                         HAL_I2C_ERROR_OVR)) != 0U) {
                self->recovery_pending_.store(true, std::memory_order_release);
            }
        }

        // Release ownership only after consuming the receive buffer.
        self->read_pending_.store(false, std::memory_order_release);
    };
}

bool AS5600::init() {
    if (recovery_task_ != nullptr) {
        return true;
    }
    if (!bus_.valid()) {
        return false;
    }
    return xTaskCreate([](void* context) {
        auto* self = static_cast<AS5600*>(context);
        for (;;) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (self->recovery_pending_.load(std::memory_order_acquire)) {
                if (!self->bus_.reset(self->transfer_)) {
                    // Another bus client owns the peripheral. Retry on a later notification.
                    continue;
                }
                self->status_.store(0U, std::memory_order_release);
                self->read_pending_.store(false, std::memory_order_release);
                self->recovery_pending_.store(false, std::memory_order_release);
            }
        }
    }, "as5600_recover", 192U, this, tskIDLE_PRIORITY + 1U,
       &recovery_task_) == pdPASS;
}

float AS5600::get_value() {
    const float rad = Encoder::get_value();
    if (recovery_task_ == nullptr) {
        return rad;
    }
    // Serialize the deadline check with the completion ISR and recovery task.
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    // 仅保留低 32 位，短超时通过无符号差值处理回绕，保持原子访问无锁。
    const uint32_t now_us = static_cast<uint32_t>(BSP::get_time_us());
    if (read_pending_.load(std::memory_order_acquire) &&
        now_us - read_started_us_.load(std::memory_order_relaxed) >= kReadTimeoutUs) {
        recovery_pending_.store(true, std::memory_order_release);
    }
    const bool recover = recovery_pending_.load(std::memory_order_acquire);
    bool expected = false;
    const bool start_read = !recover && read_pending_.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel);
    if (start_read) {
        read_started_us_.store(now_us, std::memory_order_relaxed);
    }
    __set_PRIMASK(interrupt_mask);
    if (recover) {
        if (__get_IPSR() != 0U) {
            BaseType_t task_woken = pdFALSE;
            vTaskNotifyGiveFromISR(recovery_task_, &task_woken);
            portYIELD_FROM_ISR(task_woken);
        } else {
            xTaskNotifyGive(recovery_task_);
        }
        return rad;
    }
    if (!start_read) {
        return rad;
    }

    if (!bus_.transferIt(transfer_)) {
        status_.store(0U, std::memory_order_release);
        read_pending_.store(false, std::memory_order_release);
    }
    return rad;
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
