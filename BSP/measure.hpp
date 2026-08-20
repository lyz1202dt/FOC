#pragma once

#include <cstdint>

#include "adc.h"
#include "gpio.hpp"
#include "hardware.hpp"

namespace BSP {

/**
 * @brief 相电流计算，在中断中记录record_value，在调用时计算数值
 */
class Measure {
public:
    enum ADCItemTab {
        PhraseA,
        PhraseB,
        PhraseC,
        DCBusVol,
        TabItemNum
    };
    struct {
        uint64_t time;
        uint16_t value;
    } ADCRawValue[TabItemNum];

    Measure();
    bool init();
    void set_raw_adc_value(ADCItemTab channel, uint16_t value);
    uint16_t get_raw_value(ADCItemTab channel,uint32_t time_err_us);
};

} // namespace BSP
