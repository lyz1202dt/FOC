#pragma once

#include <cstdint>

#include "adc.hpp"
#include "fdcan.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "measure.hpp"
#include "spi.hpp"
#include "svpwm.hpp"
#include "tim.hpp"
#include "usart.hpp"

namespace bsp::hardware {

extern GpioPin ic_spi_cs;
extern GpioPin sensor_spi_cs;
extern GpioPin drv8301_enable_gate;
extern GpioPin drv8301_fault;
extern GpioPin dc_cal;
extern GpioPin led;

extern SpiBus spi1;
extern SpiBus spi3;
extern I2CBus i2c1;
extern UartPort uart3;
extern FdcanBus fdcan1;
extern AdcChannel adc1;
extern Timer tim7;

// Call from a task after scheduler startup, before any bus transfers.
HAL_StatusTypeDef init();

} // namespace bsp::hardware

namespace BSP {

I2CBus& sensor_i2c_bus();
SPIBus& ic_spi_bus();
SPIBus& sensor_spi_bus();
UartPort& uart_comm_port();
FdcanBus& fdcan1_bus();
SVPWM& tim1_svpwm();
Measure& adc_measure();
bool adc_measure_start();
bool adc_measure_stop();
Gpio& ic_spi_cs();
Gpio& sensor_spi_cs();
Gpio& drv8301_enable_gate();
Gpio& drv8301_fault();
Gpio& drv8301_dc_cal();

void time_init();
uint64_t get_time_us();

} // namespace BSP

extern "C" void HardwareInit(void);
