#pragma once

#include <cstdint>

namespace BSP {

class I2CBus;
class SPIBus;

I2CBus& i2c1_bus();
SPIBus& spi1_bus();
SPIBus& spi3_bus();

void time_init();
uint32_t get_time_us();


} // namespace BSP
