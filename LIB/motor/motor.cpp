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
        //TODO:之后根据Flash配置动态选择加载其它编码器
        encoder = new AS5600();
    }
    if (!driver) {
        driver = new DRV8301(BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(),
                             BSP::drv8301_enable_gate(), BSP::drv8301_fault(), BSP::drv8301_dc_cal());
    }
    if (!controller) {
        //TODO:之后根据Flash配置动态选择加载其它控制算法
        controller = new FOC();
    }

    if (!driver->init()) {
        return false;
    }
    if (!driver->set_enable(true)) {
        return false;
    }
    motor_is_ready = driver->is_ready();
    return motor_is_ready;
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
