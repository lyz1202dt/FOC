#include "hardware.hpp"
#include "motor.hpp"
#include "stm32g431xx.h"
#include "tim.h"

extern Motor motor;

namespace BSP {

// 可注册中断回调
IRQFunc kIrqFuncTab[7];
void bind_irq_func(IQRType type, Func func, void* ctx){
    kIrqFuncTab[type].func=func;
    kIrqFuncTab[type].ctx=ctx;
}

// 高精度时钟
volatile uint32_t g_timer_update_cnt = 0U;
void time_init() {
    HAL_TIM_Base_Start_IT(&htim7); // 开启时基
}
uint32_t get_time_us() {
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    uint64_t time = ((uint64_t)__HAL_TIM_GET_COUNTER(&htim7)) + (((uint64_t)g_timer_update_cnt) << 16);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    return time;
}

extern "C" {
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    TIM_TypeDef* instance = htim->Instance;
    if (instance == TIM1) {
        if (kIrqFuncTab[TIM1_UPDATE].func)
            kIrqFuncTab[TIM1_UPDATE].func(kIrqFuncTab[TIM1_UPDATE].ctx);
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
    if (instance == ADC1) { // 注入通道，采集相电流和总线电压
        if (kIrqFuncTab[ADC1_INJEC].func)
            kIrqFuncTab[ADC1_INJEC].func(kIrqFuncTab[ADC1_INJEC].ctx);
        if (kIrqFuncTab[ADC2_INJEC].func)
            kIrqFuncTab[ADC2_INJEC].func(kIrqFuncTab[ADC2_INJEC].ctx);
        // motor.driver->set_raw_adc_value(MotorDriver::PhraseB, instance->JDR1);
        // motor.driver->set_raw_adc_value(MotorDriver::PhraseC, instance->JDR2);
        // motor.driver->set_raw_adc_value(MotorDriver::DCBusVol, instance->JDR3);
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
        if (kIrqFuncTab[SPI3_RTX].func)
            kIrqFuncTab[SPI3_RTX].func(kIrqFuncTab[SPI3_RTX].ctx);
    } else if (instance == SPI1) { // IC通信
        if (kIrqFuncTab[SPI1_RTX].func)
            kIrqFuncTab[SPI1_RTX].func(kIrqFuncTab[SPI1_RTX].ctx);
    }
}

// I2C传输完成中断
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        if (kIrqFuncTab[I2C1_MemRX].func)
            kIrqFuncTab[I2C1_MemRX].func(kIrqFuncTab[I2C1_MemRX].ctx);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c->Instance == I2C1) {
        if (kIrqFuncTab[I2C1_MemTX].func)
            kIrqFuncTab[I2C1_MemTX].func(kIrqFuncTab[I2C1_MemTX].ctx);
    }
}

} // extern C

} // namespace BSP
