#pragma once

#include "encoder.hpp"
#include "i2c.hpp"

#include <atomic>
#include "task.h"

class AS5600 : public Encoder {
public:
    struct MagnetStatus {
        bool detected;
        bool too_weak;
        bool too_strong;
    };

    // Allow angle updates with a weak magnetic field by default.
    AS5600(Flash& flash, BSP::I2CBus& bus, bool use_angle_when_magnet_weak = true);
    // Call once from a task after initializing the bus; object has application lifetime.
    bool init() override;
    // Bus must already be initialized. Requests a read and returns the cached angle.
    float get_value() override;
    // Returns false before the first successful read or after a transfer failure.
    bool GetMagnetStatus(MagnetStatus& status) const;
private:
    BSP::I2CBus& bus_;
    const bool use_angle_when_magnet_weak_;
    bsp::I2CTransfer transfer_{};
    uint8_t recv_buffer_[5]{};
    std::atomic<bool> read_pending_{false};
    std::atomic<uint8_t> status_{0U};
    std::atomic<uint32_t> read_started_ms_{0U};
    std::atomic<bool> recovery_pending_{false};
    TaskHandle_t recovery_task_ = nullptr;
};
