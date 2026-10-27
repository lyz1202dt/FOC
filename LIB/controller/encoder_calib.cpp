#include "encoder_calib.hpp"
#include "hardware.hpp"
#include <cmath>

EncoderCalib::EncoderCalib(Flash& flash)
    : Controller(flash) {
}

void EncoderCalib::reset() {
    measured_rad_       = 0.0f;
    offset_             = 0.0f;
    alignment_start_us_ = 0U;
    sample_valid_       = false;
    aligning_           = false;
    complete_           = false;
}

void EncoderCalib::set_measure(
    const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) {
    sample_valid_ = rad_is_trustable && std::isfinite(rad);
    if (sample_valid_) {
        measured_rad_ = rad;
    } else {
        aligning_ = false;
    }
}

bool EncoderCalib::get_output(float& v1, float& v2, float& v3) {
    v1 = v2 = v3 = 0.0f;
    if (complete_) {
        return true;
    }
    if (!sample_valid_) {
        return false;
    }
    const uint64_t now = BSP::get_time_us();
    if (!aligning_) {
        alignment_start_us_ = now;
        aligning_           = true;
    }
    if (now - alignment_start_us_ >= kAlignmentTimeUs) {
        offset_   = measured_rad_;
        complete_ = true;
        return true;
    }
    // Vd = 3 V, Vq = 0 V, 电角度 = 0；逆 Clarke 变换。
    v1 = 0.30f;
    v2 = v3 = -0.15f;
    return true;
}

bool EncoderCalib::get_offset(float& offset) const {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const bool complete = complete_;
    if (complete) {
        offset = offset_;
    }
    __set_PRIMASK(interrupt_mask);
    return complete;
}
