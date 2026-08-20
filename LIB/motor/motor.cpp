#include "motor.hpp"
#include "hardware.hpp"
#include "as5600.hpp"
#include "drv8301.hpp"
#include "foc.hpp"


Motor::Motor() {

}

bool Motor::init() {
    if (motor_is_ready) {
        return true;
    }

    if (!encoder) {
        encoder = new AS5600();
    }
    if (!driver) {
        driver = new DRV8301();
    }
    if (!controller) {
        controller = new FOC();
    }

    BSP::bind_irq_func(BSP::TIM1_UPDATE_IRQ,
                       [](void* ctx) {
                           static_cast<Motor*>(ctx)->update();
                       },
                       this);
    if (!driver->init()) {
        return false;
    }
    if (!driver->set_enable(true)) {
        return false;
    }
    motor_is_ready = driver->is_ready();
    return true;
}

bool Motor::is_ready()const 
{
    return motor_is_ready;
}

bool Motor::is_sensorless_mode() const{
    return sensor_less_mode;
}

bool Motor::update() {
    if (!motor_is_ready || !driver || !controller) {
        return false;
    }

    if (!driver->is_ready()) // 驱动器不可用，退出
        return false;

    float vbus_vol;
    float current[3], voltage[3];
    float rad = 0.0f;
    if (!driver->get_vbus_voltage(vbus_vol) ||
        !driver->get_phrase_current(current[0], current[1], current[2])) {
        driver->set_phrase_voltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    if (!sensor_less_mode) {
        if (!encoder->value_is_available(1000)) {
            driver->set_phrase_voltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        rad = encoder->get_value();
    }

    controller->set_measure(current[0], current[1], current[2], vbus_vol, rad);
    if (!controller->get_output(voltage[0], voltage[1], voltage[2])) {
        driver->set_phrase_voltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    if (!driver->set_phrase_voltage(voltage[0], voltage[1], voltage[2])) {
        return false;
    }
    return true;
}
