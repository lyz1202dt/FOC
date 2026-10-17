#pragma once

#include <functional>

#include "tim.hpp"

namespace BSP {

class SVPWM {
public:
    using TimerUpdateCallback = std::function<void()>;

    SVPWM(bsp::Timer& timer, bsp::PwmChannel& channel_1, bsp::PwmChannel& channel_2, bsp::PwmChannel& channel_3);

    bool init();
    void register_timer_update_cb(TimerUpdateCallback callback);
    void set_voltage(float dc_bus, float v1, float v2, float v3);
    void enable_output(bool enable);

    void timer_irq();

private:
    bsp::Timer& timer_;
    bsp::PwmChannel* channels_[3];
    TimerUpdateCallback timer_update_cb_;
};

} // namespace BSP
