#include "motor_calib.hpp"

#include "hardware.hpp"
#include <cmath>

MotorCalib::MotorCalib(Flash& flash) : Controller(flash) {}

void MotorCalib::reset() {
    stage_ = Stage::Resistance;
    result_ = {};
    stage_start_us_ = last_sample_us_ = 0U;
    current_ = vbus_ = last_current_ = current_sum_ = 0.0f;
    signed_current_change_ = signed_voltage_time_ = 0.0f;
    current_samples_ = 0U;
    applied_sign_ = 0;
    started_ = sample_valid_ = false;
}

void MotorCalib::set_measure(const float& c1, const float& c2, const float& c3, const float& vbus,
                             const float& rad, const bool& rad_is_trustable) {
    current_ = c1; // 零电角度时 Ialpha = Ia，电压矢量为 (V, -V/2, -V/2)。
    vbus_ = vbus;
    sample_valid_ = std::isfinite(c1) && std::isfinite(c2) && std::isfinite(c3) &&
                    std::isfinite(vbus) && vbus >= 6.0f && std::fabs(c1) <= 10.0f &&
                    std::fabs(c2) <= 10.0f && std::fabs(c3) <= 10.0f;
}

bool MotorCalib::get_output(float& v1, float& v2, float& v3) {
    v1 = v2 = v3 = 0.0f;
    if (stage_ == Stage::Complete) {
        return true;
    }
    if (!sample_valid_) {
        result_.success = false;
        stage_ = Stage::Complete;
        return true;
    }

    const uint64_t now = BSP::get_time_us();
    if (!started_) {
        started_ = true;
        stage_start_us_ = now;
    }
    const uint64_t elapsed = now - stage_start_us_;

    if (stage_ == Stage::Resistance) {
        if (elapsed >= kResistanceSettleUs && elapsed < kResistanceTimeUs) {
            current_sum_ += current_;
            ++current_samples_;
        }
        if (elapsed < kResistanceTimeUs) {
            v1 = 1.0f;
            v2 = v3 = -0.5f;
            return true;
        }
        const float average_current = current_samples_ != 0U ? current_sum_ / current_samples_ : 0.0f;
        result_.resistance = 1.0f / average_current;
        if (!std::isfinite(result_.resistance) || result_.resistance < 0.01f || result_.resistance > 100.0f) {
            stage_ = Stage::Complete;
            return true;
        }
        stage_ = Stage::Inductance;
        stage_start_us_ = last_sample_us_ = now;
        last_current_ = current_;
        applied_sign_ = 1;
    } else {
        const uint64_t dt_us = now - last_sample_us_;
        if (dt_us == 0U || dt_us > 200U) {
            stage_ = Stage::Complete;
            return true;
        }
        const float average_current = 0.5f * (last_current_ + current_);
        signed_current_change_ += applied_sign_ * (current_ - last_current_);
        signed_voltage_time_ += (2.0f - applied_sign_ * result_.resistance * average_current) *
                                (static_cast<float>(dt_us) * 1e-6f);
        last_current_ = current_;
        last_sample_us_ = now;
        if (elapsed >= kInductanceTimeUs) {
            result_.inductance = signed_voltage_time_ / signed_current_change_;
            result_.success = signed_current_change_ > 0.0f && std::isfinite(result_.inductance) &&
                              result_.inductance >= 2e-6f && result_.inductance <= 4e-3f;
            if (result_.success) {
                result_.kp = kBandwidth * result_.inductance;
                result_.ki = kBandwidth * result_.resistance;
            }
            stage_ = Stage::Complete;
            return true;
        }
        applied_sign_ = ((elapsed / kHalfPeriodUs) & 1U) == 0U ? 1 : -1;
    }

    v1 = 2.0f * applied_sign_;
    v2 = v3 = -0.5f * v1;
    return true;
}

bool MotorCalib::get_result(Result& result) const {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const bool complete = stage_ == Stage::Complete;
    if (complete) {
        result = result_;
    }
    __set_PRIMASK(interrupt_mask);
    return complete;
}
