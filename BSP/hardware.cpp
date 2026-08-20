#include "hardware.hpp"
#include "I2CBus.hpp"
#include "SPIBus.hpp"
#include "i2c.h"
#include "spi.h"
#include "tim.h"


namespace BSP {

namespace {

I2CBus g_i2c1_bus(&hi2c1);
SPIBus g_spi1_bus(&hspi1, IC_SPI_CS_GPIO_Port, IC_SPI_CS_Pin);
SPIBus g_spi3_bus(&hspi3, SENSOR_SPI_CS_GPIO_Port, SENSOR_SPI_CS_Pin);

} // namespace

I2CBus& i2c1_bus() {
    return g_i2c1_bus;
}

SPIBus& spi1_bus() {
    return g_spi1_bus;
}

SPIBus& spi3_bus() {
    return g_spi3_bus;
}

// 动态回调注册
IRQCallback_t kIRQCallbackTab[Tab_IRQ_NUM]{};
void bind_irq_func(IRQType type,Func func,void* ctx)
{
    kIRQCallbackTab[type].func = func;
    kIRQCallbackTab[type].ctx = ctx;
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
        if(kIRQCallbackTab[TIM1_UPDATE_IRQ].func)
            kIRQCallbackTab[TIM1_UPDATE_IRQ].func(kIRQCallbackTab[TIM1_UPDATE_IRQ].ctx);
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
        if(kIRQCallbackTab[ADC1_INJEC_IRQ].func)
            kIRQCallbackTab[ADC1_INJEC_IRQ].func(kIRQCallbackTab[ADC1_INJEC_IRQ].ctx);
    }
    else if(instance==ADC2)
    {
        if(kIRQCallbackTab[ADC2_INJEC_IRQ].func)
            kIRQCallbackTab[ADC2_INJEC_IRQ].func(kIRQCallbackTab[ADC2_INJEC_IRQ].ctx);
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
        spi3_bus().on_transfer_complete(hspi);
    } else if (instance == SPI1) { // IC通信
        spi1_bus().on_transfer_complete(hspi);
    }
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    SPI_TypeDef* instance = hspi->Instance;
    if (instance == SPI3) {
        spi3_bus().on_error(hspi);
    } else if (instance == SPI1) {
        spi1_bus().on_error(hspi);
    }
}
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_ErrorCallback(hspi);
}

// I2C传输完成中断
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_transfer_complete(hi2c);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_error(hi2c);
    }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_bus().on_error(hi2c);
    }
}

} // extern C

} // namespace BSP
