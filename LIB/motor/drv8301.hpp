#pragma once

#include <string>

#include "driver.hpp"

namespace BSP {
class SPIBus;
class SVPWM;
class Measure;
class Gpio;
}

class DRV8301 : public MotorDriver {
public:
    DRV8301(BSP::SPIBus& spi, BSP::SVPWM& svpwm, BSP::Measure& phase_current_measure,
            BSP::Gpio& enable_gate, BSP::Gpio& fault);

    bool init() override;
    bool is_ready() override;
    bool set_enable(bool enable = true) override;
    int lookup_reason(std::string& reason) override;
    
    bool get_vbus_voltage(float& vol) override;
    bool get_phrase_current(float& c1, float& c2, float& c3) override;
    bool set_phrase_voltage(float v1, float v2, float v3) override;

private:
    BSP::SPIBus& spi_;
    BSP::SVPWM& svpwm_;
    BSP::Measure& phase_current_measure_;
    BSP::Gpio& enable_gate_;
    BSP::Gpio& fault_;
    bool ready_{false};
    bool enabled_{false};
};
