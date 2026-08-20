#pragma once

#include <cstdint>

namespace BSP {

enum IRQType{
    TIM1_UPDATE_IRQ,
    ADC1_INJEC_IRQ,
    ADC2_INJEC_IRQ,
    Tab_IRQ_NUM
};

typedef void(*Func)(void* ctx);

struct IRQCallback_t{
    Func func;
    void* ctx;
};

void bind_irq_func(IRQType type,Func func,void* ctx);


class I2CBus;
class SPIBus;

I2CBus& i2c1_bus();
SPIBus& spi1_bus();
SPIBus& spi3_bus();

void time_init();
uint64_t get_time_us();


} // namespace BSP
