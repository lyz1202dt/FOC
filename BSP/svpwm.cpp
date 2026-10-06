#include "svpwm.hpp"

#include <algorithm>

namespace BSP {

struct SVPWM::Slot {
    TIM_HandleTypeDef* timer = nullptr;
    SVPWM* instance = nullptr;
};

SVPWM::Slot SVPWM::slots_[SVPWM::kMaxInstanceCount]{};

SVPWM::Slot* SVPWM::findSlot(TIM_HandleTypeDef* timer)
{
    for (Slot& slot : slots_) {
        if (slot.timer == timer) {
            return &slot;
        }
    }
    return nullptr;
}

SVPWM::Slot* SVPWM::ensureSlot(TIM_HandleTypeDef* timer)
{
    if (timer == nullptr) {
        return nullptr;
    }

    Slot* slot = findSlot(timer);
    if (slot != nullptr) {
        return slot;
    }

    for (Slot& empty_slot : slots_) {
        if (empty_slot.timer == nullptr) {
            empty_slot.timer = timer;
            return &empty_slot;
        }
    }

    return nullptr;
}

void SVPWM::periodElapsedCallback(TIM_HandleTypeDef* timer)
{
    Slot* slot = findSlot(timer);
    if (slot != nullptr && slot->instance != nullptr) {
        slot->instance->timer_irq();
    }
}

SVPWM::SVPWM(TIM_HandleTypeDef* timer, uint32_t channel_1, uint32_t channel_2, uint32_t channel_3)
    : timer_(timer)
    , channels_{channel_1, channel_2, channel_3} {
}

bool SVPWM::init() {
    if (!timer_) {
        return false;
    }

    load_config();
    set_voltage(0.0f, 0.0f, 0.0f, 0.0f);

    Slot* slot = ensureSlot(timer_);
    if (slot == nullptr) {
        return false;
    }
    slot->instance = this;
    if (HAL_TIM_RegisterCallback(timer_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &SVPWM::periodElapsedCallback) != HAL_OK) {
        slot->timer = nullptr;
        slot->instance = nullptr;
        return false;
    }

    if (HAL_TIM_PWM_Start(timer_, channels_[0]) != HAL_OK || HAL_TIMEx_PWMN_Start(timer_, channels_[0]) != HAL_OK
        || HAL_TIM_PWM_Start(timer_, channels_[1]) != HAL_OK || HAL_TIMEx_PWMN_Start(timer_, channels_[1]) != HAL_OK
        || HAL_TIM_PWM_Start(timer_, channels_[2]) != HAL_OK || HAL_TIMEx_PWMN_Start(timer_, channels_[2]) != HAL_OK) {
        slot->timer = nullptr;
        slot->instance = nullptr;
        return false;
    }

    return HAL_TIM_Base_Start_IT(timer_) == HAL_OK;
}

void SVPWM::load_config() {
}

void SVPWM::register_timer_update_cb(TimerUpdateCallback callback, void* context) {
    timer_update_cb_ = callback;
    timer_update_context_ = context;
}

void SVPWM::timer_irq() {
    if (timer_update_cb_) {
        timer_update_cb_(timer_update_context_);
    }
}

void SVPWM::set_voltage(float dc_bus, float v1, float v2, float v3) {
    if (!timer_) {
        return;
    }

    const float half_period = 0.5f * static_cast<float>(__HAL_TIM_GET_AUTORELOAD(timer_));
    if (!(dc_bus > 1e-6f)) {
        __HAL_TIM_SET_COMPARE(timer_, channels_[0], static_cast<uint32_t>(half_period));
        __HAL_TIM_SET_COMPARE(timer_, channels_[1], static_cast<uint32_t>(half_period));
        __HAL_TIM_SET_COMPARE(timer_, channels_[2], static_cast<uint32_t>(half_period));
        return;
    }

    const float max_v         = std::max(v1, std::max(v2, v3));
    const float min_v         = std::min(v1, std::min(v2, v3));
    const float zero_sequence = 0.5f * (max_v + min_v);
    const float period        = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(timer_));

    const auto to_compare = [zero_sequence, dc_bus, period](float phase_voltage) -> uint32_t {
        float duty = 0.5f + (phase_voltage - zero_sequence) / dc_bus;
        duty       = std::clamp(duty, 0.0f, 1.0f);
        return static_cast<uint32_t>(duty * period);
    };

    __HAL_TIM_SET_COMPARE(timer_, channels_[0], to_compare(v1));
    __HAL_TIM_SET_COMPARE(timer_, channels_[1], to_compare(v2));
    __HAL_TIM_SET_COMPARE(timer_, channels_[2], to_compare(v3));
}

} // namespace BSP
