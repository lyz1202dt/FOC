#pragma once

#include <cstdint>

#include "adc.h"

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
    } ADCRawValue[TabItemNum]{};

    Measure();
    bool init();
    void set_raw_adc_value(ADCItemTab channel, uint16_t value);
    bool get_raw_adc_value(ADCItemTab channel, uint16_t& value, uint32_t time_err_us);
};

} // namespace BSP
