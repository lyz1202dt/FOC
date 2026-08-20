#include "drv8301.hpp"

#include "gpio.hpp"
#include "measure.hpp"
#include "SPIBus.hpp"
#include "svpwm.hpp"

namespace {

constexpr uint32_t kSampleFreshnessUs = 10000U;
constexpr float kDefaultVbusVolts     = 24.0f;
constexpr float kDefaultVbusCounts    = 2048.0f;

} // namespace

DRV8301::DRV8301(BSP::SPIBus& spi, BSP::SVPWM& svpwm, BSP::Measure& phase_current_measure,
                 BSP::Gpio& enable_gate, BSP::Gpio& fault)
    : spi_(spi), svpwm_(svpwm), phase_current_measure_(phase_current_measure),
      enable_gate_(enable_gate), fault_(fault) {}

bool DRV8301::init() {
    ready_   = false;
    enabled_ = false;

    set_enable(false);
    set_phrase_voltage(0.0f, 0.0f, 0.0f);

    if (!phase_current_measure_.init()) {
        return false;
    }

    if (!svpwm_.init()) {
        return false;
    }

    ready_ = true;
    return true;
}

bool DRV8301::is_ready() {
    return ready_ && enabled_;
}

bool DRV8301::set_enable(bool enable) {
    enabled_ = enable;
    enable_gate_.write(enable);
    if (!enable) {
        set_phrase_voltage(0.0f, 0.0f, 0.0f);
    }
    return true;
}

int DRV8301::lookup_reason(std::string& reason) {
    if (!ready_) {
        reason = "DRV8301 not initialized";
        return -1;
    }
    if (!enabled_) {
        reason = "DRV8301 disabled";
        return 1;
    }
    if (!fault_.read()) {
        reason = "DRV8301 fault asserted";
        return -2;
    }
    reason = "OK";
    return 0;
}

bool DRV8301::get_phrase_current(float& c1, float& c2, float& c3) {

    return true;
}

bool DRV8301::get_vbus_voltage(float& vol)
{
    return true;
}

bool DRV8301::set_phrase_voltage(float v1, float v2, float v3) {
    float vbus = 0.0f;
    get_vbus_voltage(vbus);
    svpwm_.set_voltage(vbus, v1, v2, v3);
    return true;
}
