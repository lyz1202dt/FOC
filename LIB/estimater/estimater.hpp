#pragma once

#include "flash.hpp"
#include "hardware.hpp"
#include <cmath>

class Estimater {
public:
    Estimater(Flash &flash) : flash(flash) {}
    virtual ~Estimater() = default;
    virtual void set_raw_rad(float rad, bool available) {
        is_available_ = available && std::isfinite(rad);
        if (is_available_) {
            measured_rad_ = rad;
        }
    }
    virtual void set_raw_vol(float vbus, float v1, float v2, float v3) {}
    virtual void set_raw_cur(float c1, float c2, float c3) {}
    virtual bool update() {
        if (!is_available_) {
            has_previous_sample_ = false;
            omega_buf_ = 0.0f;
            return false;
        }

        const uint64_t now = BSP::get_time_us();
        if (has_previous_sample_) {
            const uint64_t elapsed_us = now - last_update_us_;
            if (elapsed_us == 0U) {
                return true;
            }
            // 最短角度差，避免编码器跨越 0/2π 时产生速度尖峰。
            constexpr float kTwoPi = 6.28318530717958647692f;
            const float delta_rad = std::remainder(measured_rad_ - rad_buf_, kTwoPi);
            const float raw_omega = delta_rad * (1000000.0f / static_cast<float>(elapsed_us));
            omega_buf_ = 0.5f * omega_buf_ + 0.5f * raw_omega;
        } else {
            omega_buf_ = 0.0f;
            has_previous_sample_ = true;
        }
        rad_buf_ = measured_rad_;
        last_update_us_ = now;
        return true;
    }
    virtual bool is_available() const {
        return is_available_;
    }

    // 返回最近一次 update 的机械角度 (rad) 和滤波后的机械角速度 (rad/s)。
    virtual bool get_value(float& rad, float& omega) const {
        rad = rad_buf_;
        omega = omega_buf_;
        return is_available() && has_previous_sample_;
    }
    uint32_t error_{0};
protected:
    Flash &flash;
    float rad_buf_{0.0f};
    float omega_buf_{0.0f};
    float measured_rad_{0.0f};
    bool is_available_{false};
    bool has_previous_sample_{false};
    uint64_t last_update_us_{0};
};
