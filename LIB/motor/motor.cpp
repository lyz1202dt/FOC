#include "motor.hpp"
#include "../BSP/hardware.hpp"


Motor::Motor() {
    driver->init();
    driver->set_enable(true);
    uint64_t start_time=BSP::get_time_us();
    while(BSP::get_time_us()-start_time<10000)
    {
        if(driver->is_ready())
            break;
        vTaskDelay(1);
    }
}

bool Motor::update() {
    if (!driver->is_ready())     //驱动器不可用，退出
        return false;

    float vbus_vol;
    float current[3], voltage[3];
    float rad = 0.0f;
    driver->get_vbus_voltage(vbus_vol);
    driver->get_phrase_current(current[0], current[1], current[2]);
    if (!sensor_less_mode) {
        if (!encoder->value_is_available(1000)) {
            driver->set_phrase_voltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        rad = encoder->get_value();
    }

    controller->set_measure(current[0], current[1], current[2], vbus_vol, rad);
    controller->get_output(voltage[0], voltage[1], voltage[2]);
    driver->set_phrase_voltage(voltage[0], voltage[1], voltage[2]);
    return true;
}
