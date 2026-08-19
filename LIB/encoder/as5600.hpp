#pragma once

#include "encoder.hpp"
#include "gpio.h"
#include "spi.h"


class AS5600 : public Encoder {
public:
    AS5600();
    void sensor_update() override;
};