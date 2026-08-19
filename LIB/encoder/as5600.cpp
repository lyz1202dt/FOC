#include "as5600.hpp"
#include "hardware.hpp"
#include "i2c.h"

AS5600::AS5600() {
    BSP::bind_irq_func(
        BSP::I2C1_MemRX,
        [](void* ctx) {
            AS5600* self = static_cast<AS5600*>(ctx);
            self->sensor_update();
        },
        this);
}

void AS5600::sensor_update() {
    HAL_I2C_Mem_Read_IT(&hi2c1, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, &recv_buffer[0], 2);
    record_value((float)((((uint16_t)(recv_buffer[0] & 0x0F)) << 8) | ((uint16_t)recv_buffer[1])) * 3.14159265f * 2.0f / 4096.0f);
}
