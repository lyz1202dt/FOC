#include "virtual.hpp"

#include "hardware.hpp"

VirtualEncoder::VirtualEncoder(Flash& flash) : Encoder(flash)
{
}

bool VirtualEncoder::init()
{
    return true;
}

bool VirtualEncoder::get_value(float& value, uint32_t max_age_us, uint64_t* sample_time_us)
{
    (void)max_age_us;
    value = 0.0f;
    if (sample_time_us != nullptr) {
        *sample_time_us = BSP::get_time_us();
    }
    return true;
}
