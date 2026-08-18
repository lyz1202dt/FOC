#pragma once

#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include <cstdint>
#include <atomic>

namespace BSP {

volatile uint32_t timer7_update_cnt(0);

void time_init() {
    HAL_TIM_Base_Start_IT(&htim7);
}

uint64_t get_time_us() {
    uint64_t value;
    __HAL_TIM_DISABLE(&htim7);
    __HAL_TIM_DISABLE_IT(&htim7,TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(&htim7);
    value=((uint32_t)__HAL_TIM_GET_COUNTER(&htim7))+((uint64_t)timer7_update_cnt<<16);
    __HAL_TIM_ENABLE_IT(&htim7,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim7);
    return value;
}


extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM7) // 定时器7 ，时基中断
    {
        timer7_update_cnt += 1;
    }
}

}

} // namespace BSP
