#include "encoder.hpp"
#include "../BSP/hardware.hpp"

Encoder::Encoder()
{

}

float Encoder::get_value()
{
    return rad;
}

bool Encoder::value_is_available(const uint32_t &us)
{
    return encoer_update_us_+us>=BSP::get_time_us();
}

void Encoder::record_value(float rad)
{
    this->rad=rad;
    encoer_update_us_=BSP::get_time_us();
}
