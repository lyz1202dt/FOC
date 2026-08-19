#include "motor.hpp"

extern Flash flash;

Motor::Motor() : encoder(nullptr), flash(::flash) {}

bool Motor::update() {
    if (!drv8301.is_ready())
        return false;

    if (encoder == nullptr || !encoder->value_is_available(1000)) {
        drv8301.disable();
        return false;
    }
    float ecc_rad = encoder->get_value(); // 得到电角度
    float c1=0.0f,c2=0.0f,c3=0.0f;
    current_measure.calc_current(c1,c2,c3);
    svpwm.set_voltage(24.0f, 0.0f, 0.0f, ecc_rad);

    return true;

}
