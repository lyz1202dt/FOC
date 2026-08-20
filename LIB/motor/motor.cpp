#include "motor.hpp"

Motor::Motor(Flash& flash, Encoder& encoder, MotorDriver& driver, Controller& controller, bool sensorless_mode)
    : flash_(flash)
    , encoder_(encoder)
    , driver_(driver)
    , controller_(controller)
    , sensorless_mode_(sensorless_mode) {
}

bool Motor::init() {
    if (motor_is_ready) {
        return true;
    }

    if (!sensorless_mode_ && !encoder_.init()) {
        return false;
    }

    if (!driver_.init()) {
        return false;
    }
    if (!driver_.set_enable(true)) {
        return false;
    }
    motor_is_ready = driver_.is_ready();
    return motor_is_ready;
}

bool Motor::is_ready()const 
{
    return motor_is_ready;
}

bool Motor::is_sensorless_mode() const{
    return sensorless_mode_;
}

bool Motor::fast_loop() {
    if (!motor_is_ready) {
        return false;
    }

    if (!driver_.is_ready()) { // 驱动器不可用，退出
        motor_is_ready = false;
        stop_output();
        return false;
    }

    float vbus_vol;
    float current[3], voltage[3];
    float rad = 0.0f;
    if (!driver_.get_vbus_voltage(vbus_vol) ||
        !driver_.get_phrase_current(current[0], current[1], current[2])) {
        stop_output();
        return false;
    }
    if (!sensorless_mode_) {
        if (!encoder_.get_value(rad, 1000)) {
            stop_output();
            return false;
        }
    }

    controller_.set_measure(current[0], current[1], current[2], vbus_vol, rad);
    if (!controller_.get_output(voltage[0], voltage[1], voltage[2])) {
        stop_output();
        return false;
    }
    if (!driver_.set_phrase_voltage(voltage[0], voltage[1], voltage[2])) {
        return false;
    }
    return true;
}

void Motor::stop_output() {
    driver_.set_phrase_voltage(0.0f, 0.0f, 0.0f);
}
