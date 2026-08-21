#pragma once

#include "uart.hpp"
#include <cstdint>

namespace BSP {

class I2CBus;
class SPIBus;
class Gpio;
class SVPWM;
class Measure;
class UARTPort;

I2CBus& sensor_i2c_bus();
SPIBus& ic_spi_bus();
SPIBus& sensor_spi_bus();
SVPWM& tim1_svpwm();
Measure& adc_measure();
Gpio& drv8301_enable_gate();
Gpio& drv8301_fault();
Gpio& drv8301_dc_cal();
UARTPort& uart_comm_port();

void time_init();
uint64_t get_time_us();


} // namespace BSP
