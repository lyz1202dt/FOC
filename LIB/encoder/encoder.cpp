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


bool Encoder::value_is_available(uint32_t max_age_us) const
{
    uint64_t sample_time = 0U;
    __disable_irq();
    sample_time = encoder_update_us_;
    __enable_irq();

    return sample_time != 0U && (BSP::get_time_us() - sample_time) <= max_age_us;
}

void Encoder::record_value(float rad)
{
    const uint64_t now = BSP::get_time_us();
    __disable_irq();
    rad_ = rad;
    encoder_update_us_ = now;
    __enable_irq();
}
