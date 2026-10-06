#pragma once

#include <cstddef>
#include <cstdint>

#include "tim.h"

namespace BSP {

class SVPWM {
public:
    using TimerUpdateCallback = void (*)(void* context);

    SVPWM(TIM_HandleTypeDef* timer, uint32_t channel_1, uint32_t channel_2, uint32_t channel_3);

    bool init();
    void load_config();
    void register_timer_update_cb(TimerUpdateCallback callback, void* context = nullptr);
    void set_voltage(float dc_bus, float v1, float v2, float v3);

    void timer_irq();

private:
    struct Slot;

    static constexpr std::size_t kMaxInstanceCount = 4U;
    static Slot slots_[kMaxInstanceCount];

    static Slot* findSlot(TIM_HandleTypeDef* timer);
    static Slot* ensureSlot(TIM_HandleTypeDef* timer);
    static void periodElapsedCallback(TIM_HandleTypeDef* timer);

    TIM_HandleTypeDef* timer_;
    uint32_t channels_[3];
    TimerUpdateCallback timer_update_cb_{nullptr};
    void* timer_update_context_{nullptr};
};

} // namespace BSP
