#include "hardware.hpp"
#include "tim.h"
#include <cstdint>

namespace BSP {

volatile uint32_t g_timer_update_cnt = 0U;

void time_init() {
    HAL_TIM_Base_Start_IT(&htim7);
}

uint32_t get_time_us() {
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    uint64_t time=((uint64_t)__HAL_TIM_GET_COUNTER(&htim7))+(((uint64_t)g_timer_update_cnt)<<16);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    return time;
}

extern "C" {
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM7) {
        g_timer_update_cnt++;
    }
}
}

} // namespace BSP
