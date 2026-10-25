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

    AS5600(Flash& flash, BSP::I2CBus& bus, bool use_angle_when_magnet_weak = true);
    bool init() override;
    bool get_value(float& value, uint32_t max_age_us, uint64_t* sample_time_us = nullptr) override;
    bool GetMagnetStatus(MagnetStatus& status) const;
private:
    BSP::I2CBus& bus_;
    const bool use_angle_when_magnet_weak_;
    bsp::I2CTransfer transfer_{};
    uint8_t recv_buffer_[5]{};
    std::atomic<bool> read_pending_{false};
    std::atomic<uint8_t> status_{0U};
    std::atomic<uint32_t> read_started_us_{0U};
    std::atomic<bool> recovery_pending_{false};
    TaskHandle_t recovery_task_ = nullptr;
};
