#include "voc.hpp"
#include "arm_math.h"
#include "hardware.hpp"
#include <cmath>

VOC::VOC(Flash& flash)
    : Controller(flash) {
}

void VOC::load_config() {
    int32_t pole_pairs      = 0;
    float mechanical_offset = 0.0f;
    flash.read_value("foc/n", pole_pairs);
    flash.read_value("foc/offset", mechanical_offset);
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    n              = pole_pairs;
    offset         = mechanical_offset;
    config_loaded_ = true;
    __set_PRIMASK(interrupt_mask);
}

void VOC::reset() {
    vir_ecc_rad         = 0.0f;
    measured_rad        = 0.0f;
    measured_rad_valid_ = false;
}

void VOC::set_measure(
    const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) {
    measured_rad_valid_ = rad_is_trustable && std::isfinite(rad);
    if (measured_rad_valid_) {
        measured_rad = rad;
    }
}

bool VOC::get_output(float& v1, float& v2, float& v3) {
    v1 = v2 = v3 = 0.0f;
    if (!config_loaded_ || (!use_virtue_rad && !measured_rad_valid_)) {
        return false;
    }
    const float vq = 1.0f; // 测试电压
    const float vd = 0.0f;

    if (use_virtue_rad) {
        vir_ecc_rad += 0.0001f;
        if (vir_ecc_rad >= 6.28318530717958647692f) {
            vir_ecc_rad = 0.0f;
        }
    } else {
        constexpr float kTwoPi = 6.28318530717958647692f;
        // offset 是校准锁存的机械角；先减机械零位，再乘极对数。
        vir_ecc_rad = std::fmod((measured_rad - offset) * static_cast<float>(n), kTwoPi);
        if (!std::isfinite(vir_ecc_rad)) {
            return false;
        }
        if (vir_ecc_rad < 0.0f) {
            vir_ecc_rad += kTwoPi;
        }
    }
    const float s = arm_sin_f32(vir_ecc_rad);
    const float c = arm_cos_f32(vir_ecc_rad);

    const float vx = c * vd - s * vq;
    const float vy = s * vd + c * vq;

    v1 = vx;
    v2 = -0.5f * vx + 0.866025404f * vy;
    v3 = -0.5f * vx - 0.866025404f * vy;

    return true;
}

void VOC::controller_reference(void* ref, size_t size) {
    if (ref == nullptr || size != sizeof(uint8_t)) {
        return;
    }
    const uint8_t mode = *static_cast<const uint8_t*>(ref);
    use_virtue_rad = mode != 0;
}
