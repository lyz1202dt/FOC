#include "app_main.hpp"
#include "hardware.hpp"
#include "svpwm.hpp"
#include "measure.hpp"
#include "I2CBus.hpp"
#include "SPIBus.hpp"
#include "tim.h"

Motor motor;

extern "C" {
void app_main(void* param) {
    BSP::time_init();
    BSP::adc_measure().init();
    BSP::ic_spi_bus().init();
    BSP::tim1_svpwm().init();
    BSP::sensor_spi_bus().init();
    BSP::sensor_i2c_bus().init();

    bool success = motor.init();
    if (!success) {
        success = motor.init();
        if (!success) {
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
    BSP::tim1_svpwm().register_timer_update_cb([]() {
        motor.update();
    });
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
}
