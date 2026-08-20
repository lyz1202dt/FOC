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

private:
    float phase_b_offset_{2048.0f};
    float phase_c_offset_{2048.0f};
    float counts_to_current_{1.0f / 2048.0f};
    uint32_t phase_b_raw_{0};
    uint32_t phase_c_raw_{0};
    bool has_sample_{false};
};
