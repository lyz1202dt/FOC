#include "encoder.hpp"
#include "hardware.hpp"
#include "stm32g4xx_hal.h"

Encoder::Encoder(Flash &flash) : flash(flash)
{

}

Encoder::~Encoder() = default;

bool Encoder::init()
{
    return true;
}

float Encoder::get_value()
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const float rad = rad_;
    __set_PRIMASK(interrupt_mask);
    return rad;
}


bool Encoder::value_is_available(uint32_t max_age_us) const
{
    uint64_t sample_time = 0U;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    sample_time = encoder_update_us_;
    __set_PRIMASK(interrupt_mask);

    return sample_time != 0U && (BSP::get_time_us() - sample_time) <= max_age_us;
}

void Encoder::record_value(float rad)
{
    const uint64_t now = BSP::get_time_us();
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    rad_ = rad;
    encoder_update_us_ = now;
    __set_PRIMASK(interrupt_mask);
}
