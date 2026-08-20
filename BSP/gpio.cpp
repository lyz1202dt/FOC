#include "gpio.hpp"

namespace BSP {

Gpio::Gpio(GPIO_TypeDef* port, uint16_t pin_mask) {
    port_     = port;
    pin_mask_ = pin_mask;
}
void Gpio::write(bool level) {
    if (!port_ || pin_mask_ == 0U) {
        return;
    }
    HAL_GPIO_WritePin(port_, pin_mask_, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
bool Gpio::read() const {
    return HAL_GPIO_ReadPin(port_, pin_mask_) == GPIO_PIN_SET;
}

} // namespace BSP
