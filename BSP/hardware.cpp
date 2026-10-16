#include "hardware.hpp"

#include "adc.h"
#include "fdcan.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

namespace bsp::hardware {

GpioPin ic_spi_cs(IC_SPI_CS_GPIO_Port, IC_SPI_CS_Pin);
GpioPin sensor_spi_cs(SENSOR_SPI_CS_GPIO_Port, SENSOR_SPI_CS_Pin);
GpioPin drv8301_enable_gate(GPIOC, GPIO_PIN_15);
GpioPin drv8301_fault(IC_FAULT_GPIO_Port, IC_FAULT_Pin);
GpioPin dc_cal(DC_CAL_GPIO_Port, DC_CAL_Pin);
GpioPin led(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

SpiBus spi1(hspi1);
SpiBus spi3(hspi3);
I2CBus i2c1(hi2c1);
UartPort uart3(huart3);
FdcanBus fdcan1(hfdcan1);
AdcChannel adc1(hadc1);
Timer tim1(htim1);
Timer tim7(htim7);
PwmChannel tim1_ch3(htim1, TIM_CHANNEL_3);
PwmChannel tim1_ch2(htim1, TIM_CHANNEL_2);
PwmChannel tim1_ch1(htim1, TIM_CHANNEL_1);

HAL_StatusTypeDef init()
{
    bool ok = spi1.init();
    ok = spi3.init() && ok;
    ok = i2c1.init() && ok;
    ok = uart3.init() && ok;
    ok = fdcan1.init() && ok;
    return ok ? HAL_OK : HAL_ERROR;
}

} // namespace bsp::hardware

namespace BSP {

namespace {
SVPWM g_tim1_svpwm(bsp::hardware::tim1, bsp::hardware::tim1_ch3, bsp::hardware::tim1_ch2, bsp::hardware::tim1_ch1);
Measure g_adc_measure;

volatile uint32_t g_timer_update_cnt = 0U;
}

I2CBus& sensor_i2c_bus()
{
    return bsp::hardware::i2c1;
}

SPIBus& ic_spi_bus()
{
    return bsp::hardware::spi1;
}

SPIBus& sensor_spi_bus()
{
    return bsp::hardware::spi3;
}

UartPort& uart_comm_port()
{
    return bsp::hardware::uart3;
}

FdcanBus& fdcan1_bus()
{
    return bsp::hardware::fdcan1;
}

SVPWM& tim1_svpwm()
{
    return g_tim1_svpwm;
}

Measure& adc_measure()
{
    return g_adc_measure;
}

bool adc_measure_start()
{
    g_adc_measure.reset();
    return bsp::hardware::adc1.injectedStartIt([]() {
        const uint64_t now = BSP::get_time_us();
        g_adc_measure.set_raw_adc_value(
            Measure::PhraseB,
            static_cast<uint16_t>(bsp::hardware::adc1.injectedValue(ADC_INJECTED_RANK_1)),
            now);
        g_adc_measure.set_raw_adc_value(
            Measure::PhraseC,
            static_cast<uint16_t>(bsp::hardware::adc1.injectedValue(ADC_INJECTED_RANK_2)),
            now);
        g_adc_measure.set_raw_adc_value(
            Measure::DCBusVol,
            static_cast<uint16_t>(bsp::hardware::adc1.injectedValue(ADC_INJECTED_RANK_3)),
            now);
    }) == HAL_OK;
}

Gpio& ic_spi_cs()
{
    return bsp::hardware::ic_spi_cs;
}

Gpio& sensor_spi_cs()
{
    return bsp::hardware::sensor_spi_cs;
}

Gpio& drv8301_enable_gate()
{
    return bsp::hardware::drv8301_enable_gate;
}

Gpio& drv8301_fault()
{
    return bsp::hardware::drv8301_fault;
}

Gpio& drv8301_dc_cal()
{
    return bsp::hardware::dc_cal;
}

void time_init()
{
    g_timer_update_cnt = 0U;
    (void)bsp::hardware::tim7.registerPeriodElapsedCallback([]() {
        ++g_timer_update_cnt;
    });
    (void)bsp::hardware::tim7.startIt();
}

uint64_t get_time_us()
{
    TIM_HandleTypeDef* tim = bsp::hardware::tim7.native();
    if (tim == nullptr) {
        return 0U;
    }

    __HAL_TIM_DISABLE_IT(tim, TIM_IT_UPDATE);
    const uint64_t time = static_cast<uint64_t>(__HAL_TIM_GET_COUNTER(tim)) +
                          (static_cast<uint64_t>(g_timer_update_cnt) << 16);
    __HAL_TIM_ENABLE_IT(tim, TIM_IT_UPDATE);
    return time;
}

} // namespace BSP

extern "C" void HardwareInit(void)
{
    (void)bsp::hardware::init();
}
