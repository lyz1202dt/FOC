#include "mit.hpp"
#include "hardware.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

MIT::MIT(Flash& flash, FOC* foc)
    : Controller(flash), foc_(foc) {
    load_config();
}

void MIT::load_config() {
    float kp = 0.0f;
    float kd = 1.0f;
    int32_t max_r = 4;
    flash.read_value("mit/kp", kp);
    flash.read_value("mit/kd", kd);
    flash.read_value("mit/max_r", max_r);
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0f || kd < 0.0f || max_r <= 0) {
        return;
    }
    if (foc_ != nullptr) {
        foc_->load_config();
    }
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    reference_.kp = kp;
    reference_.kd = kd;
    position_limit_ = static_cast<float>(max_r) * 6.28318530717958647692f;
    __set_PRIMASK(interrupt_mask);
}

void MIT::reset() {
    position_ = velocity_ = previous_rad_ = 0.0f;
    previous_us_ = 0U;
    has_sample_ = sample_valid_ = false;
    if (foc_ != nullptr) {
        foc_->reset();
        float current = 0.0f;
        foc_->controller_reference(&current, sizeof(current));
    }
}

void MIT::set_measure(
    const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad,
    const bool& rad_is_trustable) {
    sample_valid_ = foc_ != nullptr && rad_is_trustable && std::isfinite(rad)
                    && std::isfinite(c1) && std::isfinite(c2) && std::isfinite(c3)
                    && std::isfinite(vbus) && vbus > 0.0f;
    if (!sample_valid_) {
        return;
    }
    constexpr float kTwoPi = 6.28318530717958647692f;
    const uint64_t now = BSP::get_time_us();
    if (!has_sample_) {
        position_ = std::remainder(rad - foc_->offset, kTwoPi);
        velocity_ = 0.0f;
        has_sample_ = true;
    } else if (now > previous_us_) {
        const float delta = std::remainder(rad - previous_rad_, kTwoPi);
        position_ += delta;
        const float raw_velocity = delta * (1000000.0f / static_cast<float>(now - previous_us_));
        velocity_ += 0.1f * (raw_velocity - velocity_);
    }
    previous_rad_ = rad;
    previous_us_ = now;
    foc_->set_measure(c1, c2, c3, vbus, rad, true);
}

bool MIT::get_output(float& v1, float& v2, float& v3) {
    v1 = v2 = v3 = 0.0f;
    if (!sample_valid_ || !has_sample_) {
        return false;
    }
    // 只限制目标，不截断测量位置，确保越界后仍保留正确的回拉误差。
    const float target_position = std::clamp(reference_.position, -position_limit_, position_limit_);
    float current = reference_.current + reference_.kp * (target_position - position_)
                    + reference_.kd * (reference_.velocity - velocity_);
    if (!std::isfinite(current)) {
        return false;
    }
    foc_->controller_reference(&current, sizeof(current));
    return foc_->get_output(v1, v2, v3);
}

void MIT::controller_reference(void* ref, size_t size) {
    static_assert(sizeof(Reference) == 5U * sizeof(float));
    if (ref == nullptr || size != sizeof(Reference)) {
        return;
    }
    Reference reference;
    std::memcpy(&reference, ref, sizeof(reference));
    if (!std::isfinite(reference.position) || !std::isfinite(reference.velocity)
        || !std::isfinite(reference.current) || !std::isfinite(reference.kp)
        || !std::isfinite(reference.kd) || reference.kp < 0.0f || reference.kd < 0.0f) {
        return;
    }
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    reference_ = reference;
    __set_PRIMASK(interrupt_mask);
}

void MIT::controller_status(void* ref, size_t size) {
    if (foc_ != nullptr && ref != nullptr && size == sizeof(float)) {
        foc_->controller_status(ref, size);
    }
}
