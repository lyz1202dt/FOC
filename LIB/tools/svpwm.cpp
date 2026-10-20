#include "svpwm.hpp"

#include <algorithm>
#include <utility>

namespace BSP {

SVPWM::SVPWM(bsp::Timer& timer, bsp::PwmChannel& channel_1, bsp::PwmChannel& channel_2, bsp::PwmChannel& channel_3)
    : timer_(timer)
    , channels_{&channel_1, &channel_2, &channel_3} {
}

bool SVPWM::init() {
    if (initialized_) {
        return true;
    }

    if (timer_.registerPeriodElapsedCallback([this]() {
            if (timer_update_cb_) {
                timer_update_cb_();
            }
        })
        != HAL_OK) {
        return false;
    }

    set_voltage(0.0f, 0.0f, 0.0f, 0.0f);

    // 仅配置输出关闭时的低电平状态，禁止更新事件自动打开 MOE。
    TIM_TypeDef* timer = timer_.native()->Instance;
    CLEAR_BIT(timer->CR2, TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N);
    MODIFY_REG(timer->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE | TIM_BDTR_OSSI, TIM_BDTR_OSSI);
    initialized_ = true;
    return true;
}

void SVPWM::register_timer_update_cb(TimerUpdateCallback callback) {
    timer_update_cb_ = std::move(callback);
}

void SVPWM::enable_timer(bool enable) {
    TIM_TypeDef* timer            = timer_.native()->Instance;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    if (enable) {
        if ((timer->CR1 & TIM_CR1_CEN) == 0U) {
            __HAL_TIM_CLEAR_FLAG(timer_.native(), TIM_FLAG_UPDATE);
        }
        SET_BIT(timer->DIER, TIM_DIER_UIE);
        SET_BIT(timer->CR1, TIM_CR1_CEN);
    } else {
        CLEAR_BIT(timer->DIER, TIM_DIER_UIE);
        // HAL 的停止接口在 PWM 通道使能时不会清除 CEN，此处独立控制计数器。
        CLEAR_BIT(timer->CR1, TIM_CR1_CEN);
    }

    __set_PRIMASK(interrupt_mask);
}

void SVPWM::enable_output(bool enable) {
    TIM_TypeDef* timer            = timer_.native()->Instance;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    if (enable) {
        // 直接使能通道和 MOE，避免 HAL PWM 启动接口同时启动计数器。
        SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE |
                             TIM_CCER_CC2E | TIM_CCER_CC2NE |
                             TIM_CCER_CC3E | TIM_CCER_CC3NE);
        SET_BIT(timer->BDTR, TIM_BDTR_MOE);
    } else {
        // 保留通道使能，由 OSSI/OIS 将输出保持为低电平；采样时基继续运行。
        CLEAR_BIT(timer->BDTR, TIM_BDTR_MOE);
    }

    __set_PRIMASK(interrupt_mask);
}

void SVPWM::set_voltage(float dc_bus, float v1, float v2, float v3) {
    const float half_period = 0.5f * static_cast<float>(timer_.autoreload());
    if (!(dc_bus > 1e-6f)) {
        channels_[0]->setPulse(static_cast<uint32_t>(half_period));
        channels_[1]->setPulse(static_cast<uint32_t>(half_period));
        channels_[2]->setPulse(static_cast<uint32_t>(half_period));
        return;
    }

    const float max_v         = std::max(v1, std::max(v2, v3));
    const float min_v         = std::min(v1, std::min(v2, v3));
    const float zero_sequence = 0.5f * (max_v + min_v);
    const float period        = static_cast<float>(timer_.autoreload());

    const auto to_compare = [zero_sequence, dc_bus, period](float phase_voltage) -> uint32_t {
        float duty = 0.5f + (phase_voltage - zero_sequence) / dc_bus;
        duty       = std::clamp(duty, 0.0f, 1.0f);
        return static_cast<uint32_t>(duty * period);
    };

    channels_[0]->setPulse(to_compare(v1));
    channels_[1]->setPulse(to_compare(v2));
    channels_[2]->setPulse(to_compare(v3));
}

} // namespace BSP
