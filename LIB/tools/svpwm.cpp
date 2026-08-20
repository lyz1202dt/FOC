#include "svpwm.hpp"

#include <algorithm>

#include "tim.h"


SVPWM::SVPWM() {
}
void SVPWM::load_config() {
}
void SVPWM::set_voltage(float dc_bus, float v1,float v2,float v3) {
    const float half_period = 0.5f * static_cast<float>(__HAL_TIM_GET_AUTORELOAD(&htim1));
    if (!(dc_bus > 1e-6f)) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(half_period));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, static_cast<uint32_t>(half_period));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, static_cast<uint32_t>(half_period));
        return;
    }

    const float max_v = std::max(v1, std::max(v2, v3));
    const float min_v = std::min(v1, std::min(v2, v3));
    const float zero_sequence = 0.5f * (max_v + min_v);
    const float period = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(&htim1));

    const auto to_compare = [zero_sequence, dc_bus, period](float phase_voltage) -> uint32_t {
        float duty = 0.5f + (phase_voltage - zero_sequence) / dc_bus;
        duty = std::clamp(duty, 0.0f, 1.0f);
        return static_cast<uint32_t>(duty * period);
    };

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, to_compare(v1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, to_compare(v2));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, to_compare(v3));
}
