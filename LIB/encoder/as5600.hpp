#pragma once

#include "encoder.hpp"
#include "i2c.hpp"

class AS5600 : public Encoder {
public:
    explicit AS5600(BSP::I2CBus& bus);
    bool init() override;
    bool sensor_update();
private:
    static void on_read_complete(void* ctx, bool success);

    BSP::I2CBus& bus_;
    bsp::I2CTransfer transfer_{};
    uint8_t recv_buffer[2]{};
    bool started_{false};
};
