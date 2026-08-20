#pragma once

#include <cstdint>
#include <functional>

#include "tim.h"

namespace BSP {

class SVPWM {
public:
    SVPWM(TIM_HandleTypeDef* timer, uint32_t channel_1, uint32_t channel_2, uint32_t channel_3);

    bool init();
    void load_config();
    void register_timer_update_cb(std::function<void(void)> func);
    void set_voltage(float dc_bus, float v1, float v2, float v3);

    void timer_irq();

private:
    TIM_HandleTypeDef* timer_;
    uint32_t channels_[3];
    std::function<void(void)> timer_update_cb_;
};

} // namespace BSP
