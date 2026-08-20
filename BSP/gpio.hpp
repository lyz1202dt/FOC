#pragma once

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

namespace BSP {

class Gpio {
public:
    Gpio(GPIO_TypeDef* port, uint16_t pin_mask);
    void write(bool level);
    bool read() const;

private:
    GPIO_TypeDef* port_;
    uint16_t pin_mask_;
};

} // namespace BSP
