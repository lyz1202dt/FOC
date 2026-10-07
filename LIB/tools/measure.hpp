#pragma once

#include <cstdint>

namespace tools {

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

    Measure() = default;

    void reset();
    void set_raw_adc_value(ADCItemTab channel, uint16_t value, uint64_t time_us);
    bool get_raw_adc_value(ADCItemTab channel, uint16_t& value) const;
    bool get_raw_adc_value(ADCItemTab channel, uint16_t& value, uint32_t time_err_us, uint64_t now_us) const;
};

} // namespace tools

namespace BSP {
using Measure = tools::Measure;
} // namespace BSP
