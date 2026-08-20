#include "encoder.hpp"
#include "hardware.hpp"

Encoder::Encoder()
{

}

float Encoder::get_value()
{
    return rad;
}

bool Encoder::value_is_available(const uint32_t &us)
{
    return (BSP::get_time_us() - encoder_update_us_) <= us;
}

void Encoder::record_value(float rad)
{
    this->rad=rad;
    encoder_update_us_=BSP::get_time_us();
}
