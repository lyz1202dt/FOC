#include "measure.hpp"

namespace BSP {

Measure::Measure() {
}

bool Measure::init() {
    return true;
}

void Measure::set_raw_adc_value(ADCItemTab channel, uint16_t value)
{
    ADCRawValue[channel].value = value;
    ADCRawValue[channel].time  = BSP::get_time_us();
}


} // namespace BSP
