#pragma once

#include "encoder.hpp"
#include "../BSP/hardware.hpp"


class AS5600 : public Encoder {
public:
    AS5600();
    void sensor_update();
    uint8_t recv_buffer[2];
};
