#include "svpwm.hpp"

#include <algorithm>
#include <utility>

namespace BSP {

SVPWM::SVPWM(bsp::Timer& timer, bsp::PwmChannel& channel_1, bsp::PwmChannel& channel_2, bsp::PwmChannel& channel_3)
    : timer_(timer)
    , channels_{&channel_1, &channel_2, &channel_3} {
}

bool SVPWM::init() {
    set_voltage(0.0f, 0.0f, 0.0f, 0.0f);

    if (timer_.registerPeriodElapsedCallback([this]() {
            timer_irq();
        }) != HAL_OK) {
        return false;
    }

    if (channels_[0]->start() != HAL_OK || channels_[0]->startComplementary() != HAL_OK
        || channels_[1]->start() != HAL_OK || channels_[1]->startComplementary() != HAL_OK
        || channels_[2]->start() != HAL_OK || channels_[2]->startComplementary() != HAL_OK) {
        return false;
    }

    return timer_.startIt() == HAL_OK;
}

void SVPWM::register_timer_update_cb(TimerUpdateCallback callback) {
    timer_update_cb_ = std::move(callback);
}

void SVPWM::timer_irq() {
    if (timer_update_cb_) {
        timer_update_cb_();
    }
}

void SVPWM::enable_output(bool enable) {
    TIM_TypeDef* timer = timer_.native()->Instance;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    // Drive all three main/complementary outputs low when MOE is cleared.
    // Keep the channels and counter enabled for update IRQs and ADC triggers.
    CLEAR_BIT(timer->CR2, TIM_CR2_OIS1 | TIM_CR2_OIS1N
                             | TIM_CR2_OIS2 | TIM_CR2_OIS2N
                             | TIM_CR2_OIS3 | TIM_CR2_OIS3N);
    // Disable automatic re-enabling on update events while output is off.
    MODIFY_REG(timer->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE,
               TIM_BDTR_OSSI | (enable ? TIM_BDTR_MOE : 0U));

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
