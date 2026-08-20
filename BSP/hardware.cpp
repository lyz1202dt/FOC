#include "hardware.hpp"
#include "gpio.hpp"
#include "I2CBus.hpp"
#include "measure.hpp"
#include "SPIBus.hpp"
#include "svpwm.hpp"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"


namespace BSP {

namespace {
//GPIO设备
Gpio g_ic_spi_cs(IC_SPI_CS_GPIO_Port, IC_SPI_CS_Pin);
Gpio g_sensor_spi_cs(SENSOR_SPI_CS_GPIO_Port, SENSOR_SPI_CS_Pin);
Gpio g_drv8301_enable_gate(GPIOC, GPIO_PIN_15);
Gpio g_drv8301_fault(IC_FAULT_GPIO_Port, IC_FAULT_Pin);
Gpio g_dc_cal(DC_CAL_GPIO_Port, DC_CAL_Pin);
//总线设备
I2CBus g_sensor_i2c_bus(&hi2c1);
SPIBus g_ic_spi_bus(&hspi1, &g_ic_spi_cs);
SPIBus g_sensor_spi_bus(&hspi3, &g_sensor_spi_cs);
//板载硬件设备
SVPWM g_tim1_svpwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
Measure g_adc_measure;

} // namespace

I2CBus& sensor_i2c_bus() {
    return g_sensor_i2c_bus;
}

SPIBus& ic_spi_bus() {
    return g_ic_spi_bus;
}

SPIBus& sensor_spi_bus() {
    return g_sensor_spi_bus;
}

SVPWM& tim1_svpwm() {
    return g_tim1_svpwm;
}

Measure& adc_measure() {
    return g_adc_measure;
}

Gpio& drv8301_enable_gate() {
    return g_drv8301_enable_gate;
}

Gpio& drv8301_fault() {
    return g_drv8301_fault;
}


// 高精度时钟
volatile uint32_t g_timer_update_cnt = 0U;
void time_init() {
    HAL_TIM_Base_Start_IT(&htim7); // 开启时基
}
uint64_t get_time_us() {
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    uint64_t time = ((uint64_t)__HAL_TIM_GET_COUNTER(&htim7)) + (((uint64_t)g_timer_update_cnt) << 16);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    return time;
}


//中断处理
extern "C" {
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    TIM_TypeDef* instance = htim->Instance;
    if (instance == TIM1) {
        g_tim1_svpwm.timer_irq();
    }
    if (instance == TIM7) {
        g_timer_update_cnt++;
    }
}

// ADC普通采集中断
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // 电流采样完成，执行FOC计算
{
}

// ADC注入通道数据采集中断
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    ADC_TypeDef* instance = hadc->Instance;
    if(instance==ADC1)
    {
        g_adc_measure.set_raw_adc_value(Measure::PhraseB, instance->JDR1);
        g_adc_measure.set_raw_adc_value(Measure::PhraseC, instance->JDR2);
        g_adc_measure.set_raw_adc_value(Measure::DCBusVol, instance->JDR3);
    }
    else if(instance==ADC2)
    {
        
    }
}

// SPI传输中断
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_TxRxCpltCallback(hspi);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    SPI_TypeDef* instance = hspi->Instance;
    if (instance == SPI3) {
        sensor_spi_bus().on_transfer_complete(hspi);
    } else if (instance == SPI1) { // IC通信
        ic_spi_bus().on_transfer_complete(hspi);
    }
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    SPI_TypeDef* instance = hspi->Instance;
    if (instance == SPI3) {
        sensor_spi_bus().on_error(hspi);
    } else if (instance == SPI1) {
        ic_spi_bus().on_error(hspi);
    }
}
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_ErrorCallback(hspi);
}

// I2C传输完成中断
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_error(hi2c);
    }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        sensor_i2c_bus().on_error(hi2c);
    }
}

} // extern C

} // namespace BSP
