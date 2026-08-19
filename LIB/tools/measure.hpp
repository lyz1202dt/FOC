#pragma once

#include <tuple>
#include <cstdint>

/**
* @brief 相电流计算，在中断中记录record_value，在调用时计算数值
 */
class PhaseCurrentMeasure{
public:
    PhaseCurrentMeasure();
    void load_config();
    void record_adc_value(uint32_t ch2,uint32_t ch3);
    void calc_current(float &c1,float &c2,float &c3);
};
