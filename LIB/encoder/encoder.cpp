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

bool Encoder::get_value(float& value, uint32_t max_age_us, uint64_t* sample_time_us)
{
    uint64_t sample_time = 0U;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    value = rad_;
    sample_time = encoder_update_us_;
    __set_PRIMASK(interrupt_mask);

    if (sample_time_us != nullptr) {
        *sample_time_us = sample_time;
    }
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
