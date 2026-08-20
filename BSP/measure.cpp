#include "measure.hpp"
#include "hardware.hpp"
#include "stm32g4xx_hal_adc_ex.h"

namespace BSP {

Measure::Measure() {
}

bool Measure::init() {
    return HAL_ADCEx_InjectedStart_IT(&hadc1) == HAL_OK;
}

void Measure::set_raw_adc_value(ADCItemTab channel, uint16_t value)
{
    if (channel >= TabItemNum) {
        return;
    }

    ADCRawValue[channel].value = value;
    ADCRawValue[channel].time  = BSP::get_time_us();
}

bool Measure::get_raw_adc_value(ADCItemTab channel, uint16_t &value,uint32_t time_err_us)
{
    if (channel >= TabItemNum) {
        return false;
    }

    uint64_t sample_time = 0U;
    uint16_t sample_value = 0U;
    __disable_irq();
    sample_time  = ADCRawValue[channel].time;
    sample_value = ADCRawValue[channel].value;
    __enable_irq();

    if (sample_time == 0U || sample_time + time_err_us < BSP::get_time_us()) {
        return false;
    }

    value = sample_value;
    return true;
}
} // namespace BSP
