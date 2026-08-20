#include "measure.hpp"

PhaseCurrentMeasure::PhaseCurrentMeasure()
{
    load_config();
}

void PhaseCurrentMeasure::load_config()
{
    phase_b_offset_ = 2048.0f;
    phase_c_offset_ = 2048.0f;
    counts_to_current_ = 1.0f / 2048.0f;
    phase_b_raw_ = 0U;
    phase_c_raw_ = 0U;
    has_sample_ = false;
}

void PhaseCurrentMeasure::record_adc_value(uint32_t ch2,uint32_t ch3)
{
    phase_b_raw_ = ch2;
    phase_c_raw_ = ch3;
    has_sample_ = true;
}

void PhaseCurrentMeasure::calc_current(float &c1,float &c2,float &c3)
{
    if (!has_sample_) {
        c1 = 0.0f;
        c2 = 0.0f;
        c3 = 0.0f;
        return;
    }

    const float current_b = (static_cast<float>(phase_b_raw_) - phase_b_offset_) * counts_to_current_;
    const float current_c = (static_cast<float>(phase_c_raw_) - phase_c_offset_) * counts_to_current_;
    c2 = current_b;
    c3 = current_c;
    c1 = -(current_b + current_c);
}
