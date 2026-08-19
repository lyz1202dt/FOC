#include "hardware.hpp"
#include "tim.h"
#include "motor.hpp"

extern Motor motor;

namespace BSP {

volatile uint32_t g_timer_update_cnt = 0U;

void time_init() {
    HAL_TIM_Base_Start_IT(&htim7);      //开启时基

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
//普通采集
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) // 电流采样完成，执行FOC计算
{
    if (hadc->Instance == ADC1) // TODO:对采集到的电流进行滤波
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

//注入通道数据采集
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        /* JDR1, JDR2, JDR3 分别对应 Rank 1, 2, 3 的数据寄存器 */
        uint32_t value_ch1 = hadc->Instance->JDR1;
        uint32_t value_ch2 = hadc->Instance->JDR2;
        uint32_t value_ch3 = hadc->Instance->JDR3;
        
        // ... 处理数据 ...
    }
}

}

} // namespace BSP
