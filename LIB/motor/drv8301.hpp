#pragma once

#include <string>

#include "driver.hpp"
#include "measure.hpp"
#include "svpwm.hpp"

class DRV8301 : public MotorDriver {
public:
    DRV8301();

    bool init() override;
    bool is_ready() override;
    bool set_enable(bool enable = true) override;
    int lookup_reason(std::string& reason) override;
    bool get_vbus_voltage(float& vol) override;
    bool get_phrase_current(float& c1, float& c2, float& c3) override;
    bool set_phrase_voltage(float v1, float v2, float v3) override;

private:
    bool adc_sample_fresh(ADCMotor channel, uint32_t max_age_us) const;

    PhaseCurrentMeasure phase_current_measure_;
    SVPWM svpwm_;
    bool ready_{false};
    bool enabled_{false};
};
