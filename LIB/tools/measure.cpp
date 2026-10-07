#include "measure.hpp"

#include "stm32g4xx_hal.h"

namespace tools {

void Measure::reset()
{
    __disable_irq();
    for (auto& item : ADCRawValue) {
        item.time = 0U;
        item.value = 0U;
    }
    __enable_irq();
}

void Measure::set_raw_adc_value(ADCItemTab channel, uint16_t value, uint64_t time_us)
{
    if (channel >= TabItemNum) {
        return;
    }

    ADCRawValue[channel].value = value;
    ADCRawValue[channel].time = time_us;
}

bool Measure::get_raw_adc_value(ADCItemTab channel, uint16_t& value) const
{
    if (channel >= TabItemNum) {
        return false;
    }

    uint64_t sample_time = 0U;
    uint16_t sample_value = 0U;
    __disable_irq();
    sample_time = ADCRawValue[channel].time;
    sample_value = ADCRawValue[channel].value;
    __enable_irq();

    if (sample_time == 0U) {
        return false;
    }

    value = sample_value;
    return true;
}

bool Measure::get_raw_adc_value(ADCItemTab channel, uint16_t& value, uint32_t time_err_us, uint64_t now_us) const
{
    if (channel >= TabItemNum) {
        return false;
    }

    uint64_t sample_time = 0U;
    uint16_t sample_value = 0U;
    __disable_irq();
    sample_time = ADCRawValue[channel].time;
    sample_value = ADCRawValue[channel].value;
    __enable_irq();

    if (sample_time == 0U || sample_time + time_err_us < now_us) {
        return false;
    }

    value = sample_value;
    return true;
}

} // namespace tools
