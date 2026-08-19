#pragma once

#include <cstdint>

namespace BSP {

enum IQRType{
    TIM1_UPDATE,
    ADC1_INJEC,
    ADC2_INJEC,
    SPI1_RTX,
    SPI3_RTX,
    I2C1_MemRX,
    I2C1_MemTX,
};

typedef void(*Func)(void* ctx);

struct IRQFunc{
    Func func;
    void* ctx;
};

void bind_irq_func(IQRType type, Func func, void* ctx);

void time_init();
uint32_t get_time_us();


} // namespace BSP
