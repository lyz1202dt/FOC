#pragma once

#include "encoder.hpp"


class AS5600 : public Encoder {
public:
    AS5600();
    void sensor_update();
private:
    static void on_read_complete(void* ctx, bool success);

    uint8_t recv_buffer[2]{};
};
