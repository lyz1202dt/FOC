#pragma once

#include <functional>

#include "tim.hpp"

namespace BSP {

class SVPWM {
public:
    using TimerUpdateCallback = std::function<void()>;

    SVPWM(bsp::Timer& timer, bsp::PwmChannel& channel_1, bsp::PwmChannel& channel_2, bsp::PwmChannel& channel_3);

    bool init();  // 初始化占空比、输出空闲状态和更新回调，不启动计数器或 PWM 输出。
    void register_timer_update_cb(TimerUpdateCallback callback);
    void set_voltage(float dc_bus, float v1, float v2, float v3);
    void enable_timer(bool enable);   // 仅控制计数器和更新中断。
    void enable_output(bool enable);  // 仅控制 PWM 输出，不改变计数器和更新中断。

private:
    bsp::Timer& timer_;
    bsp::PwmChannel* channels_[3];
    TimerUpdateCallback timer_update_cb_;
    bool initialized_{false};
};

} // namespace BSP
