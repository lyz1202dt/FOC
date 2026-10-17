#pragma once

#include "encoder.hpp"
#include "i2c.hpp"

#include <atomic>

class AS5600 : public Encoder {
public:
    struct MagnetStatus {
        bool detected;
        bool too_weak;
        bool too_strong;
    };

    // Allow angle updates with a weak magnetic field by default.
    AS5600(Flash& flash, BSP::I2CBus& bus, bool use_angle_when_magnet_weak = true);
    // Bus must already be initialized. Starts continuous asynchronous sampling.
    bool init() override;
    // Restarts sampling after a transfer failure; false while a read is pending.
    bool sensor_update();
    // Returns false before the first successful read or after a transfer failure.
    bool GetMagnetStatus(MagnetStatus& status) const;
private:
    static void on_read_complete(void* ctx, bool success);

    BSP::I2CBus& bus_;
    const bool use_angle_when_magnet_weak_;
    bsp::I2CTransfer transfer_{};
    uint8_t recv_buffer_[5]{};
    std::atomic<bool> read_pending_{false};
    std::atomic<uint8_t> status_{0U};
};
